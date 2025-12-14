#include "ICBS.h"
#include <queue>
#include <algorithm>
#include <unordered_set>
#include <unordered_map>
#include <cmath>
#include <limits>
#include <map>

namespace Core
{

// ==================== Constructor ====================
ICBS::ICBS(int _row, int _col,
           std::vector<std::pair<int, int>> _edges,
           std::vector<std::tuple<int, int, int, int>> _agents,
           int _mergeThreshold,
           bool _mergeRestartActive, uint32_t _maxMetaAgentSize)
    : row(_row), col(_col), edges(_edges),
      agents(_agents), mergeThreshold(_mergeThreshold),
      mergeRestartActive(_mergeRestartActive), maxMetaAgentSize(_maxMetaAgentSize), 
      llSolver(_row, _col, _edges, {})
{
    int V = row * col;
    g.resize(V);
    for (int id = 0; id < int(edges.size()); id++) {
        auto [u, v] = edges[id];
        g[u].push_back(id);
        g[v].push_back(id);
    }

    countAgent = static_cast<int>(agents.size());
    initMerges.resize(countAgent);
    for (int i = 0; i < countAgent; i++) {
        initMerges[i] = {i};
    }
}

// ==================== Utility Functions ====================
std::pair<int, int> ICBS::toCoord(int v) {
    return {v / col, v % col};
}

int ICBS::heuristic(int u, int goal) {
    auto [ux, uy] = toCoord(u);
    auto [gx, gy] = toCoord(goal);
    return std::abs(ux - gx) + std::abs(uy - gy);
}

int ICBS::getPosition(const Path& path, int time) {
    if (time < 0) return path.empty() ? -1 : path[0];
    if (time >= static_cast<int>(path.size())) return path.empty() ? -1 : path.back();
    return path[time];
}

std::vector<Path> ICBS::extractSolution(Ref<ICTNode> node) {
    std::vector<Path> result(countAgent);
    for (const auto& ma : node->metaAgents) {
        for (size_t i = 0; i < ma.agents.size(); ++i) {
            result[ma.agents[i]] = ma.solution[i];
        }
    }
    return result;
}

// ==================== Cost Functions (Time Window) ====================

double ICBS::agentCS(int agent, const Path& path) {
    auto [start, goal, et, lt] = agents[agent];
    int arrivalTime = static_cast<int>(path.size()) - 1;
    return customerSatisfaction(arrivalTime, et, lt);
}

double ICBS::metaAgentCS(const MetaAgent& ma) {
    double totalCS = 0.0;
    for (size_t i = 0; i < ma.agents.size(); ++i) {
        totalCS += agentCS(ma.agents[i], ma.solution[i]);
    }
    return totalCS;
}

double ICBS::calculateCost(Ref<ICTNode> node) {
    double totalCS = 0.0;
    for (const auto& ma : node->metaAgents) {
        totalCS += ma.cost;
    }
    return totalCS;
}

// ==================== MDD Functions ====================

// Build MDD from scratch
MDD ICBS::buildMDD(int agent, int pathLength, const MetaAgentConstraints& constraints) {
    auto [start, goal, et, lt] = agents[agent];
    MDD mdd;
    mdd.levels.resize(pathLength + 1);
    
    std::vector<std::unordered_set<int>> reachable(pathLength + 1);
    reachable[0].insert(start);
    
    // Forward pass: find reachable vertices
    for (int t = 0; t < pathLength; ++t) {
        for (int v : reachable[t]) {
            // Wait at current vertex
            if (!isBlocked(constraints, v, t + 1)) {
                reachable[t + 1].insert(v);
            }
            // Move to adjacent vertices
            for (int edgeIdx : g[v]) {
                auto [u1, u2] = edges[edgeIdx];
                int nv = (u1 == v) ? u2 : u1;
                if (!isBlocked(constraints, nv, t + 1) &&
                    !isEdgeBlocked(constraints, v, nv, t)) {
                    reachable[t + 1].insert(nv);
                }
            }
        }
    }
    
    // Backward pass: filter to optimal paths only
    std::vector<std::unordered_set<int>> onOptimalPath(pathLength + 1);
    if (reachable[pathLength].contains(goal)) {
        onOptimalPath[pathLength].insert(goal);
    }
    
    for (int t = pathLength - 1; t >= 0; --t) {
        for (int v : reachable[t]) {
            // Check if waiting leads to optimal path
            if (onOptimalPath[t + 1].contains(v) && !isBlocked(constraints, v, t + 1)) {
                onOptimalPath[t].insert(v);
            }
            // Check if moving leads to optimal path
            for (int edgeIdx : g[v]) {
                auto [u1, u2] = edges[edgeIdx];
                int nv = (u1 == v) ? u2 : u1;
                if (onOptimalPath[t + 1].contains(nv) &&
                    !isBlocked(constraints, nv, t + 1) &&
                    !isEdgeBlocked(constraints, v, nv, t)) {
                    onOptimalPath[t].insert(v);
                }
            }
        }
    }
    
    // Copy to MDD structure
    for (int t = 0; t <= pathLength; ++t) {
        for (int v : onOptimalPath[t]) {
            mdd.levels[t].push_back(v);
        }
    }
    return mdd;
}

// Build MDD incrementally from parent's MDD
MDD ICBS::buildMDDFrom(int agent, int pathLength, 
                       const MetaAgentConstraints& constraints,
                       const MDD& parentMDD) {
    auto [start, goal, et, lt] = agents[agent];
    MDD mdd;
    mdd.levels.resize(pathLength + 1);
    
    // Nếu parent MDD rỗng hoặc không hợp lệ, build từ đầu
    if (parentMDD.levels.empty() || 
        parentMDD.levels.size() != static_cast<size_t>(pathLength + 1)) {
        return buildMDD(agent, pathLength, constraints);
    }
    
    std::vector<std::unordered_set<int>> reachable(pathLength + 1);
    
    // Forward pass: filter parent MDD with new constraints
    for (int t = 0; t <= pathLength; ++t) {
        for (int v : parentMDD.levels[t]) {
            if (t == 0) {
                if (v == start) {
                    reachable[0].insert(v);
                }
            } else {
                // Check if v is still reachable from previous level
                bool canReach = false;
                
                // Check waiting from same vertex
                if (reachable[t - 1].contains(v) && !isBlocked(constraints, v, t)) {
                    canReach = true;
                }
                
                // Check moving from adjacent vertices
                if (!canReach) {
                    for (int edgeIdx : g[v]) {
                        auto [u1, u2] = edges[edgeIdx];
                        int prev = (u1 == v) ? u2 : u1;
                        
                        if (reachable[t - 1].contains(prev) &&
                            !isBlocked(constraints, v, t) &&
                            !isEdgeBlocked(constraints, prev, v, t - 1)) {
                            canReach = true;
                            break;
                        }
                    }
                }
                
                if (canReach) {
                    reachable[t].insert(v);
                }
            }
        }
    }
    
    // Backward pass: keep only vertices on optimal paths to goal
    std::vector<std::unordered_set<int>> onOptimalPath(pathLength + 1);
    if (reachable[pathLength].contains(goal)) {
        onOptimalPath[pathLength].insert(goal);
    }
    
    for (int t = pathLength - 1; t >= 0; --t) {
        for (int v : reachable[t]) {
            // Check if waiting leads to optimal path
            if (onOptimalPath[t + 1].contains(v) && !isBlocked(constraints, v, t + 1)) {
                onOptimalPath[t].insert(v);
            }
            // Check if moving leads to optimal path
            for (int edgeIdx : g[v]) {
                auto [u1, u2] = edges[edgeIdx];
                int nv = (u1 == v) ? u2 : u1;
                if (onOptimalPath[t + 1].contains(nv) &&
                    !isBlocked(constraints, nv, t + 1) &&
                    !isEdgeBlocked(constraints, v, nv, t)) {
                    onOptimalPath[t].insert(v);
                }
            }
        }
    }
    
    // Copy to MDD structure
    for (int t = 0; t <= pathLength; ++t) {
        for (int v : onOptimalPath[t]) {
            mdd.levels[t].push_back(v);
        }
    }
    return mdd;
}

int ICBS::getMDDWidth(const MDD& mdd, int time) {
    if (time < 0 || time >= static_cast<int>(mdd.levels.size())) return 0;
    return static_cast<int>(mdd.levels[time].size());
}

// ==================== Conflict Detection ====================
void ICBS::processConflict(Ref<ICTNode> node, int ma1, int ma2) {
    int smaller = std::min(ma1, ma2);
    int larger = std::max(ma1, ma2);
    
    const auto& metaAgent1 = node->metaAgents[smaller];
    const auto& metaAgent2 = node->metaAgents[larger];
    
    std::optional<IConflict> firstConflict = std::nullopt;
    int conflictCount = 0;
    
    int maxTime = 0;
    for (const auto& path : metaAgent1.solution)
        maxTime = std::max(maxTime, static_cast<int>(path.size()));
    for (const auto& path : metaAgent2.solution)
        maxTime = std::max(maxTime, static_cast<int>(path.size()));
    
    for (size_t i = 0; i < metaAgent1.agents.size(); ++i) {
        for (size_t j = 0; j < metaAgent2.agents.size(); ++j) {
            const Path& p1 = metaAgent1.solution[i];
            const Path& p2 = metaAgent2.solution[j];
            
            for (int t = 0; t < maxTime; ++t) {
                int pos1 = getPosition(p1, t);
                int pos2 = getPosition(p2, t);
                
                if (pos1 == pos2 && pos1 != -1) {
                    conflictCount++;
                    if (!firstConflict) {
                        firstConflict = IConflict{smaller, larger, pos1, t, false, -1, ConflictType::NONE};
                    }
                }
                
                if (t > 0) {
                    int prev1 = getPosition(p1, t - 1);
                    int prev2 = getPosition(p2, t - 1);
                    if (pos1 == prev2 && pos2 == prev1 && pos1 != pos2) {
                        conflictCount++;
                        if (!firstConflict) {
                            firstConflict = IConflict{smaller, larger, prev1, t - 1, true, pos1, ConflictType::NONE};
                        }
                    }
                }
            }
        }
    }
    
    int oldCumulative = std::get<2>(node->conflictMatrix.conflicts[larger][smaller]);
    node->conflictMatrix.conflicts[larger][smaller] = {
        firstConflict,
        conflictCount,
        oldCumulative + (firstConflict.has_value() ? 1 : 0)
    };
}

void ICBS::findAllConflicts(Ref<ICTNode> node) {
    int n = static_cast<int>(node->metaAgents.size());
    node->conflictMatrix.resize(n);
    
    for (int i = 1; i < n; ++i) {
        for (int j = 0; j < i; ++j) {
            processConflict(node, j, i);
        }
    }
    node->numConflicts = computeTotalConflicts(node);
}

int ICBS::computeTotalConflicts(Ref<ICTNode> node) {
    int total = 0;
    int n = static_cast<int>(node->metaAgents.size());
    for (int i = 1; i < n; ++i) {
        for (int j = 0; j < i; ++j) {
            total += std::get<1>(node->conflictMatrix.conflicts[i][j]);
        }
    }
    return total;
}

int ICBS::countMetaAgentConflicts(Ref<ICTNode> node, int metaAgent) {
    int count = 0;
    int n = static_cast<int>(node->metaAgents.size());
    
    for (int i = 0; i < n; ++i) {
        if (i == metaAgent) continue;
        int larger = std::max(i, metaAgent);
        int smaller = std::min(i, metaAgent);
        count += std::get<1>(node->conflictMatrix.conflicts[larger][smaller]);
    }
    return count;
}

// ==================== Update MetaAgent ====================
void ICBS::updateMetaAgent(Ref<ICTNode> node, int metaAgent, const std::vector<Path>& newPaths) {
    auto& ma = node->metaAgents[metaAgent];
    int n = static_cast<int>(node->metaAgents.size());
    
    int oldConflicts = countMetaAgentConflicts(node, metaAgent);
    node->numConflicts -= oldConflicts;
    
    int oldLen = ma.totalPathLength;
    int newLen = 0;
    for (const auto& p : newPaths) {
        newLen += static_cast<int>(p.size());
    }
    node->totalPathLength += (newLen - oldLen);
    
    double oldCS = ma.cost;
    ma.solution = newPaths;
    ma.totalPathLength = newLen;
    ma.cost = 0;
    for (size_t i = 0; i < ma.agents.size(); ++i) {
        ma.cost += agentCS(ma.agents[i], newPaths[i]);
    }
    node->cost += (ma.cost - oldCS);
    
    for (int i = 0; i < n; ++i) {
        if (i == metaAgent) continue;
        processConflict(node, i, metaAgent);
    }
    
    int newConflicts = countMetaAgentConflicts(node, metaAgent);
    node->numConflicts += newConflicts;
}

// ==================== Conflict Classification (PC) ====================
ConflictType ICBS::classifyConflict(Ref<ICTNode> node, const IConflict& conflict) {
    const auto& ma1 = node->metaAgents[conflict.metaAgent1];
    const auto& ma2 = node->metaAgents[conflict.metaAgent2];
    
    if (ma1.agents.size() == 1 && ma2.agents.size() == 1) {
        int agent1 = ma1.agents[0];
        int agent2 = ma2.agents[0];
        int len1 = static_cast<int>(ma1.solution[0].size()) - 1;
        int len2 = static_cast<int>(ma2.solution[0].size()) - 1;
        
        // Tái sử dụng MDD từ cache nếu có
        MDD mdd1, mdd2;
        
        if (node->mddCache.contains(agent1)) {
            mdd1 = buildMDDFrom(agent1, len1, 
                               node->constraints[conflict.metaAgent1],
                               node->mddCache.get(agent1));
        } else {
            mdd1 = buildMDD(agent1, len1, node->constraints[conflict.metaAgent1]);
        }
        
        if (node->mddCache.contains(agent2)) {
            mdd2 = buildMDDFrom(agent2, len2, 
                               node->constraints[conflict.metaAgent2],
                               node->mddCache.get(agent2));
        } else {
            mdd2 = buildMDD(agent2, len2, node->constraints[conflict.metaAgent2]);
        }
        
        int width1 = getMDDWidth(mdd1, conflict.time);
        int width2 = getMDDWidth(mdd2, conflict.time);
        
        if (width1 == 1 && width2 == 1) return ConflictType::CARDINAL;
        if (width1 == 1) return ConflictType::SEMI_CARDINAL1;
        if (width2 == 1) return ConflictType::SEMI_CARDINAL2;
        return ConflictType::NON_CARDINAL;
    }
    
    auto child1 = generateChild(node, conflict.metaAgent1, conflict);
    auto child2 = generateChild(node, conflict.metaAgent2, conflict);
    
    bool cost1Increased = child1 && (*node < *child1);
    bool cost2Increased = child2 && (*node < *child2);
    
    if (cost1Increased && cost2Increased) return ConflictType::CARDINAL;
    if (cost1Increased) return ConflictType::SEMI_CARDINAL1;
    if (cost2Increased) return ConflictType::SEMI_CARDINAL2;
    return ConflictType::NON_CARDINAL;
}

std::optional<IConflict> ICBS::selectConflict(Ref<ICTNode> node) {
    std::optional<IConflict> semiCardinal_single, nonCardinal_single;
    std::optional<IConflict> semiCardinal_multi, nonCardinal_multi;
    
    int n = static_cast<int>(node->metaAgents.size());
    
    // Vòng 1: Duyệt các cặp single-agent (1-1) để tìm cardinal
    for (int i = 1; i < n; ++i) {
        for (int j = 0; j < i; ++j) {
            if (node->metaAgents[i].agents.size() != 1 || 
                node->metaAgents[j].agents.size() != 1) {
                continue;
            }
            
            auto conflictOpt = std::get<0>(node->conflictMatrix.conflicts[i][j]);
            if (!conflictOpt.has_value()) continue;
            
            IConflict conflict = *conflictOpt;
            conflict.type = classifyConflict(node, conflict);
            
            if (conflict.type == ConflictType::CARDINAL) {
                return conflict; // Cardinal từ single-agent - ưu tiên cao nhất
            } else if (conflict.type == ConflictType::SEMI_CARDINAL1 || 
                       conflict.type == ConflictType::SEMI_CARDINAL2) {
                if (!semiCardinal_single.has_value()) semiCardinal_single = conflict;
            } else {
                if (!nonCardinal_single.has_value()) nonCardinal_single = conflict;
            }
        }
    }
    
    // Vòng 2: Duyệt các cặp multi-agent để tìm cardinal
    for (int i = 1; i < n; ++i) {
        for (int j = 0; j < i; ++j) {
            if (node->metaAgents[i].agents.size() == 1 && 
                node->metaAgents[j].agents.size() == 1) {
                continue;
            }
            
            auto conflictOpt = std::get<0>(node->conflictMatrix.conflicts[i][j]);
            if (!conflictOpt.has_value()) continue;
            
            IConflict conflict = *conflictOpt;
            conflict.type = classifyConflict(node, conflict);
            
            if (conflict.type == ConflictType::CARDINAL) {
                return conflict; // Cardinal từ multi-agent
            } else if (conflict.type == ConflictType::SEMI_CARDINAL1 || 
                       conflict.type == ConflictType::SEMI_CARDINAL2) {
                if (!semiCardinal_multi.has_value()) semiCardinal_multi = conflict;
            } else {
                if (!nonCardinal_multi.has_value()) nonCardinal_multi = conflict;
            }
        }
    }
    
    // Trả về theo thứ tự ưu tiên
    if (semiCardinal_single.has_value()) return semiCardinal_single;
    if (semiCardinal_multi.has_value()) return semiCardinal_multi;
    if (nonCardinal_single.has_value()) return nonCardinal_single;
    return nonCardinal_multi;
}

// ==================== Low-Level Search ====================
std::optional<std::vector<Path>> ICBS::lowLevelSearch(int metaAgent, Ref<ICTNode> node) {
    const auto& ma = node->metaAgents[metaAgent];
    const auto& constraints = node->constraints[metaAgent];
    
    if (ma.agents.size() == 1) {
        int agent = ma.agents[0];
        auto [start, goal, et, lt] = agents[agent];
        
        std::priority_queue<AStarState, std::vector<AStarState>, std::greater<>> open;
        std::map<std::pair<int,int>, int> bestG;
        std::map<std::pair<int,int>, std::pair<int,int>> parent;
        
        open.push({start, 0, 0, heuristic(start, goal)});
        bestG[{start, 0}] = 0;
        
        int maxTime = lt + row * col;
        
        while (!open.empty()) {
            auto [v, t, gVal, fVal] = open.top();
            open.pop();
            
            if (t > maxTime) continue;
            
            if (v == goal && !isGoalBlockedAfter(constraints, goal, t)) {
                Path path;
                int cv = v, ct = t;
                while (ct > 0 || cv != start) {
                    path.push_back(cv);
                    auto [pv, pt] = parent[{cv, ct}];
                    cv = pv; ct = pt;
                }
                path.push_back(start);
                std::reverse(path.begin(), path.end());
                return std::vector<Path>{path};
            }
            
            if (bestG.count({v, t}) && bestG[{v, t}] < gVal) continue;
            
            // Wait
            if (!isBlocked(constraints, v, t + 1)) {
                int newG = gVal + 1;
                if (!bestG.count({v, t + 1}) || newG < bestG[{v, t + 1}]) {
                    bestG[{v, t + 1}] = newG;
                    open.push({v, t + 1, newG, newG + heuristic(v, goal)});
                    parent[{v, t + 1}] = {v, t};
                }
            }
            
            // Move
            for (int edgeIdx : g[v]) {
                auto [u1, u2] = edges[edgeIdx];
                int nv = (u1 == v) ? u2 : u1;
                
                if (isBlocked(constraints, nv, t + 1)) continue;
                if (isEdgeBlocked(constraints, v, nv, t)) continue;
                
                int newG = gVal + 1;
                if (!bestG.count({nv, t + 1}) || newG < bestG[{nv, t + 1}]) {
                    bestG[{nv, t + 1}] = newG;
                    open.push({nv, t + 1, newG, newG + heuristic(nv, goal)});
                    parent[{nv, t + 1}] = {v, t};
                }
            }
        }
        return std::nullopt;
    }
    
    // Meta-agent with multiple agents: use CBS
    std::vector<std::tuple<int, int, int, int>> subAgents;
    for (int a : ma.agents) {
        subAgents.push_back(agents[a]);
    }
    
    llSolver.setAgents(std::move(subAgents));
    llSolver.setExtraConstraints(constraints);
    llSolver.setTimeLimit(timeRemain());
    llSolver.solve(pythonSignals);
    
    if (llSolver.result.found) {
        return llSolver.result.solution;
    }
    return std::nullopt;
}

// ==================== Bypass ====================
bool ICBS::tryBypass(Ref<ICTNode> node, const IConflict& conflict) {
    if (conflict.type == ConflictType::CARDINAL) return false;
    
    int originalNC = node->numConflicts;
    
    for (int maIdx : {conflict.metaAgent1, conflict.metaAgent2}) {
        // Create temporary node for testing
        Ref<ICTNode> tempNode = CreateRef<ICTNode>(*node);
        
        Constraint newConstraint{conflict.vertex, conflict.time, conflict.isEdge, conflict.vertex2};
        tempNode->constraints[maIdx].push_back(newConstraint);
        
        auto newPaths = lowLevelSearch(maIdx, tempNode);
        if (!newPaths.has_value()) continue;
        
        double oldAgentCS = tempNode->metaAgents[maIdx].cost;
        double newAgentCS = 0;
        for (size_t i = 0; i < tempNode->metaAgents[maIdx].agents.size(); ++i) {
            newAgentCS += agentCS(tempNode->metaAgents[maIdx].agents[i], (*newPaths)[i]);
        }
        
        if (std::abs(newAgentCS - oldAgentCS) < 1e-9) {
            updateMetaAgent(tempNode, maIdx, *newPaths);
            
            if (tempNode->numConflicts < originalNC) {
                // Apply changes to original node
                *node = *tempNode;
                return true;
            }
        }
    }
    return false;
}

// ==================== MA-CBS ====================
bool ICBS::shouldMerge(Ref<ICTNode> node, int ma1, int ma2) {
    if (node->metaAgents[ma1].agents.size() + node->metaAgents[ma2].agents.size() > maxMetaAgentSize) return false;
    int larger = std::max(ma1, ma2);
    int smaller = std::min(ma1, ma2);
    int cumulativeConflicts = std::get<2>(node->conflictMatrix.conflicts[larger][smaller]);
    return cumulativeConflicts >= mergeThreshold;
}

Ref<ICTNode> ICBS::merge(Ref<ICTNode> node, int ma1, int ma2) {
    Ref<ICTNode> newNode = CreateRef<ICTNode>(*node);
    
    int keepIdx = std::min(ma1, ma2);
    int removeIdx = std::max(ma1, ma2);
    
    auto& keepMA = newNode->metaAgents[keepIdx];
    const auto& removeMA = newNode->metaAgents[removeIdx];
    
    // Merge agents and solutions
    for (int a : removeMA.agents) {
        keepMA.agents.push_back(a);
    }
    for (const auto& p : removeMA.solution) {
        keepMA.solution.push_back(p);
    }
    keepMA.cost += removeMA.cost;
    keepMA.totalPathLength += removeMA.totalPathLength;
    
    // Merge constraints
    for (const auto& c : newNode->constraints[removeIdx]) {
        newNode->constraints[keepIdx].push_back(c);
    }
    
    // Remove merged meta-agent
    newNode->metaAgents.erase(newNode->metaAgents.begin() + removeIdx);
    newNode->constraints.erase(newNode->constraints.begin() + removeIdx);
    newNode->conflictMatrix.remove(removeIdx);
    
    // Recompute conflicts for keepIdx
    int n = static_cast<int>(newNode->metaAgents.size());
    for (int i = 0; i < n; ++i) {
        if (i == keepIdx) continue;
        processConflict(newNode, i, keepIdx);
    }
    newNode->numConflicts = computeTotalConflicts(newNode);
    
    return newNode;
}

// ==================== Generate Child ====================
Ref<ICTNode> ICBS::generateChild(Ref<ICTNode> parent, int metaAgent, const IConflict& conflict) {
    Ref<ICTNode> child = CreateRef<ICTNode>(*parent);
    
    Constraint newConstraint{conflict.vertex, conflict.time, conflict.isEdge, conflict.vertex2};
    child->constraints[metaAgent].push_back(newConstraint);
    
    auto newPaths = lowLevelSearch(metaAgent, child);
    if (!newPaths.has_value()) return nullptr;
    
    updateMetaAgent(child, metaAgent, *newPaths);
    
    // Cache MDD cho các agent đơn lẻ trong meta-agent này
    const auto& ma = child->metaAgents[metaAgent];
    if (ma.agents.size() == 1) {
        int agent = ma.agents[0];
        int pathLen = static_cast<int>(ma.solution[0].size()) - 1;
        
        // Build và cache MDD
        MDD mdd;
        if (parent->mddCache.contains(agent)) {
            mdd = buildMDDFrom(agent, pathLen, 
                              child->constraints[metaAgent],
                              parent->mddCache.get(agent));
        } else {
            mdd = buildMDD(agent, pathLen, child->constraints[metaAgent]);
        }
        child->mddCache.set(agent, std::move(mdd));
    }
    
    return child;
}
// ==================== High-Level Search ====================
ICBSResult ICBS::highLevelSearch(bool restarted) {
    if (!restarted) startTime = std::chrono::steady_clock::now();
    
    Ref<ICTNode> root = CreateRef<ICTNode>();
    root->totalPathLength = 0;
    root->cost = 0;
    root->numConflicts = 0;
    
    // Create meta-agents
    root->metaAgents.clear();
    root->constraints.clear();
    root->conflictMatrix.clear();

    root->metaAgents.reserve(initMerges.size());
    root->constraints.reserve(initMerges.size());
    for (auto& agentList : initMerges) {
        MetaAgent ma;
        ma.agents = agentList;
        ma.cost = 0;
        ma.totalPathLength = 0;
        root->metaAgents.push_back(std::move(ma));
        root->constraints.push_back({});
    }

    // Find initial paths for each meta-agent
    for (int i = 0; i < static_cast<int>(root->metaAgents.size()); ++i) {
        auto paths = lowLevelSearch(i, root);
        if (!paths.has_value() || paths->empty()) {
            return {false, {}, 0};
        }
        auto& ma = root->metaAgents[i];
        ma.solution = *paths;
        ma.cost = 0;
        ma.totalPathLength = 0;
        for (size_t j = 0; j < ma.agents.size(); ++j) {
            ma.cost += agentCS(ma.agents[j], (*paths)[j]);
            ma.totalPathLength += static_cast<int>((*paths)[j].size());
        }
        root->cost += ma.cost;
        root->totalPathLength += ma.totalPathLength;
    }
    
    findAllConflicts(root);

    // Lambda comparator for priority queue
    auto cmp = [](Ref<ICTNode> a, Ref<ICTNode> b) {
        return *a < *b;
    };
    
    std::priority_queue<Ref<ICTNode>, std::vector<Ref<ICTNode>>, decltype(cmp)> open(cmp);
    open.push(root);

    while (!open.empty()) {
        if (isTimeUp()) {
            return {false, {}, 0};
        }
        
        Ref<ICTNode> current = open.top();
        open.pop();

        if (current->numConflicts == 0) {
            auto solution = extractSolution(current);
            return {true, solution, current->cost};
        }
        
        auto conflictOpt = selectConflict(current);
        if (!conflictOpt.has_value()) continue;
        
        IConflict conflict = *conflictOpt;
        
        // Try bypass for non-cardinal conflicts
        if (conflict.type != ConflictType::CARDINAL) {
            if (tryBypass(current, conflict)) {
                open.push(current);
                continue;
            }
        }
        
        // Check merge threshold
        if (shouldMerge(current, conflict.metaAgent1, conflict.metaAgent2)) {
            if (mergeRestartActive) {
                // Record merge for restart
                for (auto v : initMerges[conflict.metaAgent1]) {
                    initMerges[conflict.metaAgent2].push_back(v);
                }
                initMerges.erase(initMerges.begin() + conflict.metaAgent1);

                // Restart search
                return highLevelSearch(true);
            } else {
                // Local merge
                Ref<ICTNode> merged = merge(current, conflict.metaAgent1, conflict.metaAgent2);
                int mergedIdx = std::min(conflict.metaAgent1, conflict.metaAgent2);
                auto newPaths = lowLevelSearch(mergedIdx, merged);
                if (newPaths.has_value()) {
                    updateMetaAgent(merged, mergedIdx, *newPaths);
                    open.push(merged);
                }
                continue;
            }
        }
        
        // Split: generate two children
        auto child1 = generateChild(current, conflict.metaAgent1, conflict);
        auto child2 = generateChild(current, conflict.metaAgent2, conflict);
        
        if (child1) open.push(child1);
        if (child2) open.push(child2);
    }
    
    return {false, {}, 0};
}

}