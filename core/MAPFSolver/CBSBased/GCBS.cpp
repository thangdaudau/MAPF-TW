#include "GCBS.h"
#include <queue>
#include <map>
#include <iostream>
#include <algorithm>
#include <set>

namespace Core {

GCBS::GCBS(int _row, int _col, 
           std::vector<std::pair<int, int>> _edges, 
           std::vector<std::tuple<int, int, int, int>> _agents,
           GCBSMode _mode,
           ConflictHeuristic _heuristicType)
: CBS(_row, _col, std::move(_edges), std::move(_agents))
, mode(_mode)
, heuristicType(_heuristicType)
, alternatingCounter(0)
{
}

CBSResult GCBS::highLevelSearch()
{
    startTime = std::chrono::steady_clock::now();
    
    // Lambda comparators for each heuristic
    // Note: cost tie-breaking uses < because we're maximizing CS (higher cost = better)
    auto cmp_h1 = [](Ref<CTNodeH> a, Ref<CTNodeH> b) {
        if (a->h1 != a->h1) return a->h1 > a->h1;
        if (std::abs(a->base->cost - a->base->cost) > 1e-9) return a->base->cost < a->base->cost;  // Higher cost is better
        return a->base->totalPathLength > a->base->totalPathLength;
    };
    
    auto cmp_h2 = [](Ref<CTNodeH> a, Ref<CTNodeH> b) {
        if (a->h2 != a->h2) return a->h2 > a->h2;
        if (std::abs(a->base->cost - a->base->cost) > 1e-9) return a->base->cost < a->base->cost;
        return a->base->totalPathLength > a->base->totalPathLength;
    };
    
    auto cmp_h3 = [](Ref<CTNodeH> a, Ref<CTNodeH> b) {
        if (a->h3 != a->h3) return a->h3 > a->h3;
        if (std::abs(a->base->cost - a->base->cost) > 1e-9) return a->base->cost < a->base->cost;
        return a->base->totalPathLength > a->base->totalPathLength;
    };
    
    auto cmp_h4 = [this](Ref<CTNodeH> a, Ref<CTNodeH> b) {
        int h4a = computeVertexCover(a);
        int h4b = computeVertexCover(b);
        if (h4a != h4b) return h4a > h4b;
        if (std::abs(a->base->cost - b->base->cost) > 1e-9) return a->base->cost < b->base->cost;
        return a->base->totalPathLength > b->base->totalPathLength;
    };
    
    // Four separate OPEN lists for alternating
    std::priority_queue<Ref<CTNodeH>, std::vector<Ref<CTNodeH>>, decltype(cmp_h1)> open_h1(cmp_h1);
    std::priority_queue<Ref<CTNodeH>, std::vector<Ref<CTNodeH>>, decltype(cmp_h2)> open_h2(cmp_h2);
    std::priority_queue<Ref<CTNodeH>, std::vector<Ref<CTNodeH>>, decltype(cmp_h3)> open_h3(cmp_h3);
    std::priority_queue<Ref<CTNodeH>, std::vector<Ref<CTNodeH>>, decltype(cmp_h4)> open_h4(cmp_h4);
    
    // Build root node
    Ref<CTNodeH> root = CreateRef<CTNodeH>();
    root->base = CreateRef<CTNode>();
    root->base->constraints.resize(count_agent);
    root->base->solution.resize(count_agent);
    root->init(count_agent);
    
    // Choose low-level search based on mode
    bool useConflictAvoidance = (mode == GCBSMode::GCBS_L || mode == GCBSMode::GCBS_LH);
    
    for (int i = 0; i < count_agent; i++) {
        std::optional<Path> path;
        if (useConflictAvoidance && i > 0) {
            path = lowLevelSearchWithConflictAvoidance(i, root->base->constraints[i], root->base->solution);
        } else {
            path = lowLevelSearch(i, root->base->constraints[i]);
        }
        if (!path) return {false, {}, 0};
        root->base->solution[i] = *path;
    }
    
    root->base->cost = calculateCost(root->base->solution);
    root->base->numConflicts = computeTotalConflicts(root->base->solution);
    root->base->totalPathLength = 0;
    for (const auto& p : root->base->solution) {
        root->base->totalPathLength += p.size();
    }
    
    // Build CAT for root
    for (int i = 0; i < count_agent; i++) {
        for (int j = i + 1; j < count_agent; j++) {
            int maxLen = std::max(root->base->solution[i].size(), root->base->solution[j].size());
            auto getPos = [&](int agent, int t) -> int {
                if (t < (int)root->base->solution[agent].size()) return root->base->solution[agent][t];
                return root->base->solution[agent].back();
            };
            
            int conflicts = 0;
            for (int t = 0; t < maxLen; t++) {
                if (getPos(i, t) == getPos(j, t)) conflicts++;
            }
            for (int t = 1; t < maxLen; t++) {
                int pi0 = getPos(i, t-1), pi1 = getPos(i, t);
                int pj0 = getPos(j, t-1), pj1 = getPos(j, t);
                if (pi0 == pj1 && pi1 == pj0 && pi0 != pi1) conflicts++;
            }
            
            if (conflicts > 0) {
                root->conflictMatrix[i][j] = root->conflictMatrix[j][i] = true;
                root->conflictCount[i][j] = root->conflictCount[j][i] = conflicts;
            }
        }
    }
    root->computeHeuristics(count_agent);
    
    // Add root to appropriate OPEN lists
    if (heuristicType == ConflictHeuristic::ALTERNATING || heuristicType == ConflictHeuristic::NUM_CONFLICTS)
        open_h1.push(root);
    if (heuristicType == ConflictHeuristic::ALTERNATING || heuristicType == ConflictHeuristic::NUM_CONFLICTING_AGENTS)
        open_h2.push(root);
    if (heuristicType == ConflictHeuristic::ALTERNATING || heuristicType == ConflictHeuristic::NUM_CONFLICTING_PAIRS)
        open_h3.push(root);
    if (heuristicType == ConflictHeuristic::ALTERNATING || heuristicType == ConflictHeuristic::VERTEX_COVER)
        open_h4.push(root);
    
    auto hasNodes = [&]() {
        if (heuristicType == ConflictHeuristic::ALTERNATING) {
            return !open_h1.empty() || !open_h2.empty() || !open_h3.empty() || !open_h4.empty();
        }
        switch (heuristicType) {
            case ConflictHeuristic::NUM_CONFLICTS: return !open_h1.empty();
            case ConflictHeuristic::NUM_CONFLICTING_AGENTS: return !open_h2.empty();
            case ConflictHeuristic::NUM_CONFLICTING_PAIRS: return !open_h3.empty();
            case ConflictHeuristic::VERTEX_COVER: return !open_h4.empty();
            default: return false;
        }
    };
    
    int uid = 1;
    // std::set<int> closed;
    
    auto getNextNode = [&]() -> Ref<CTNodeH> {
        if (heuristicType == ConflictHeuristic::ALTERNATING) {
            while (!open_h1.empty() || !open_h2.empty() || !open_h3.empty() || !open_h4.empty()) {
                int idx = alternatingCounter % 4;
                alternatingCounter++;
                
                if (idx == 0 && !open_h1.empty()) {
                    Ref<CTNodeH> node = open_h1.top();
                    open_h1.pop();
                    // if (closed.contains(node->id)) continue;
                    return node;
                } else if (idx == 1 && !open_h2.empty()) {
                    Ref<CTNodeH> node = open_h2.top();
                    open_h2.pop();
                    // if (closed.contains(node->id)) continue;
                    return node;
                } else if (idx == 2 && !open_h3.empty()) {
                    Ref<CTNodeH> node = open_h3.top();
                    open_h3.pop();
                    // if (closed.contains(node->id)) continue;
                    return node;
                } else if (idx == 3 && !open_h4.empty()) {
                    Ref<CTNodeH> node = open_h4.top();
                    open_h4.pop();
                    // if (closed.contains(node->id)) continue;
                    return node;
                }
            }
            return nullptr;
        } else {
            switch (heuristicType) {
                case ConflictHeuristic::NUM_CONFLICTS:
                    if (!open_h1.empty()) {
                        Ref<CTNodeH> node = open_h1.top();
                        open_h1.pop();
                        return node;
                    }
                    break;
                case ConflictHeuristic::NUM_CONFLICTING_AGENTS:
                    if (!open_h2.empty()) {
                        Ref<CTNodeH> node = open_h2.top();
                        open_h2.pop();
                        return node;
                    }
                    break;
                case ConflictHeuristic::NUM_CONFLICTING_PAIRS:
                    if (!open_h3.empty()) {
                        Ref<CTNodeH> node = open_h3.top();
                        open_h3.pop();
                        return node;
                    }
                    break;
                case ConflictHeuristic::VERTEX_COVER:
                    if (!open_h4.empty()) {
                        Ref<CTNodeH> node = open_h4.top();
                        open_h4.pop();
                        return node;
                    }
                    break;
                default:
                    break;
            }
            return nullptr;
        }
    };
    
    auto addNode = [&](Ref<CTNodeH> node) {
        node->id = uid;
        uid++;
        if (heuristicType == ConflictHeuristic::ALTERNATING) {
            int idx = (alternatingCounter + 3) % 4;
            switch (idx) {
                case 0: open_h1.push(node); break;
                case 1: open_h2.push(node); break;
                case 2: open_h3.push(node); break;
                case 3: open_h4.push(node); break;
            }
        } else {
            switch (heuristicType) {
                case ConflictHeuristic::NUM_CONFLICTS:
                    open_h1.push(node);
                    break;
                case ConflictHeuristic::NUM_CONFLICTING_AGENTS:
                    open_h2.push(node);
                    break;
                case ConflictHeuristic::NUM_CONFLICTING_PAIRS:
                    open_h3.push(node);
                    break;
                case ConflictHeuristic::VERTEX_COVER:
                    open_h4.push(node);
                    break;
                default:
                    break;
            }
        }
    };
    
    while (hasNodes()) {
        if (isTimeUp()) {
            return {false, {}, 0};
        }
        
        Ref<CTNodeH> cur = getNextNode();
        if (!cur) break;
        
        auto conflict = findFirstConflict(cur->base->solution);
        
        if (!conflict) {
            return {true, cur->base->solution, cur->base->cost};
        }
        
        // Generate children
        for (int ai : {conflict->agent1, conflict->agent2}) {
            Ref<CTNodeH> child = CreateRef<CTNodeH>();
            
            // Copy base CTNode data
            child->base = CreateRef<CTNode>();
            child->base->constraints = cur->base->constraints;
            child->base->solution = cur->base->solution;
            child->base->cost = cur->base->cost;
            child->base->numConflicts = cur->base->numConflicts;
            child->base->totalPathLength = cur->base->totalPathLength;
            
            // Copy CAT data
            child->conflictMatrix = cur->conflictMatrix;
            child->conflictCount = cur->conflictCount;
            child->h1 = cur->h1;
            child->h2 = cur->h2;
            child->h3 = cur->h3;
            
            child->base->constraints[ai].push_back({conflict->vertex, conflict->time, 
                                                   conflict->isEdge, conflict->vertex2});
            
            // Find new path
            std::optional<Path> newPath;
            if (mode == GCBSMode::GCBS_L || mode == GCBSMode::GCBS_LH) {
                newPath = lowLevelSearchWithConflictAvoidance(ai, child->base->constraints[ai], child->base->solution);
            } else {
                newPath = lowLevelSearch(ai, child->base->constraints[ai]);
            }

            if (!newPath) continue;
            
            updateAgentPathH(child, ai, *newPath);
            
            addNode(child);
        }
        // closed.insert(cur->id);
    }
    return {false, {}, 0};
}

void GCBS::updateAgentPathH(Ref<CTNodeH> node, int agent, const Path& newPath)
{
    // Update cost
    double oldCS = agentCS(agent, node->base->solution[agent]);
    double newCS = agentCS(agent, newPath);
    node->base->cost += (newCS - oldCS);
    
    node->base->totalPathLength += (int)newPath.size() - (int)node->base->solution[agent].size();
    
    // Update path
    node->base->solution[agent] = newPath;
    
    // Rebuild CAT
    for (int i = 0; i < count_agent; i++) {
        for (int j = i + 1; j < count_agent; j++) {
            int maxLen = std::max(node->base->solution[i].size(), node->base->solution[j].size());
            auto getPos = [&](int a, int t) -> int {
                if (t < (int)node->base->solution[a].size()) return node->base->solution[a][t];
                return node->base->solution[a].back();
            };
            
            int conflicts = 0;
            for (int t = 0; t < maxLen; t++) {
                if (getPos(i, t) == getPos(j, t)) conflicts++;
            }
            for (int t = 1; t < maxLen; t++) {
                int pi0 = getPos(i, t-1), pi1 = getPos(i, t);
                int pj0 = getPos(j, t-1), pj1 = getPos(j, t);
                if (pi0 == pj1 && pi1 == pj0 && pi0 != pi1) conflicts++;
            }
            
            node->conflictMatrix[i][j] = node->conflictMatrix[j][i] = (conflicts > 0);
            node->conflictCount[i][j] = node->conflictCount[j][i] = conflicts;
        }
    }
    
    node->computeHeuristics(count_agent);
    node->base->numConflicts = node->h1;
}

Ref<CTNodeH> GCBS::buildCTNodeH(Ref<CTNode> ctNode)
{
    Ref<CTNodeH> nodeH = CreateRef<CTNodeH>();
    nodeH->base = ctNode;
    nodeH->init(count_agent);
    
    // Build CAT
    for (int i = 0; i < count_agent; i++) {
        for (int j = i + 1; j < count_agent; j++) {
            int maxLen = std::max(ctNode->solution[i].size(), ctNode->solution[j].size());
            auto getPos = [&](int agent, int t) -> int {
                if (t < (int)ctNode->solution[agent].size()) return ctNode->solution[agent][t];
                return ctNode->solution[agent].back();
            };
            
            int conflicts = 0;
            for (int t = 0; t < maxLen; t++) {
                if (getPos(i, t) == getPos(j, t)) conflicts++;
            }
            for (int t = 1; t < maxLen; t++) {
                int pi0 = getPos(i, t-1), pi1 = getPos(i, t);
                int pj0 = getPos(j, t-1), pj1 = getPos(j, t);
                if (pi0 == pj1 && pi1 == pj0 && pi0 != pi1) conflicts++;
            }
            
            if (conflicts > 0) {
                nodeH->conflictMatrix[i][j] = nodeH->conflictMatrix[j][i] = true;
                nodeH->conflictCount[i][j] = nodeH->conflictCount[j][i] = conflicts;
            }
        }
    }
    
    nodeH->computeHeuristics(count_agent);
    return nodeH;
}

int GCBS::computeVertexCover(Ref<CTNodeH> node)
{
    std::vector<std::vector<int>> g(count_agent);
    
    for (int i = 0; i < count_agent; i++) {
        for (int j = 0; j < count_agent; j++) {
            if (i != j && node->conflictMatrix[i][j]) {
                g[i].push_back(j);
            }
        }
    }
    
    std::vector<int> coverSet = Core::MinimumVertexCover(g);
    
    return coverSet.size();
}

std::optional<Path> GCBS::lowLevelSearchWithConflictAvoidance(
    int agent, 
    const AgentConstraints& constraints,
    const std::vector<Path>& currentSolution)
{
    auto [start, goal, et, lt] = agents[agent];
    
    const int CONFLICT_PENALTY = 10000;
    
    struct AStarStateConflict {
        int vertex, time, g;
        double cost;
        double cs;
        
        bool operator>(const AStarStateConflict& o) const {
            if (std::abs(cost - o.cost) > 1e-9) return cost > o.cost;
            return cs < o.cs;
        }
    };
    
    std::priority_queue<AStarStateConflict, std::vector<AStarStateConflict>, std::greater<>> open;
    std::map<std::pair<int,int>, int> bestG;
    std::map<std::pair<int,int>, std::pair<int,int>> parent;
    
    auto hasConflict = [&](int v, int t) -> bool {
        for (int j = 0; j < count_agent; j++) {
            if (j == agent) continue;
            if (currentSolution[j].empty()) continue;
            
            int pos = (t < (int)currentSolution[j].size()) ? currentSolution[j][t] : currentSolution[j].back();
            if (pos == v) return true;
        }
        return false;
    };
    
    auto hasEdgeConflict = [&](int v1, int v2, int t) -> bool {
        for (int j = 0; j < count_agent; j++) {
            if (j == agent) continue;
            if (currentSolution[j].empty()) continue;
            
            if (t == 0 || t >= (int)currentSolution[j].size()) continue;
            
            int prev = currentSolution[j][t-1];
            int curr = currentSolution[j][t];
            
            if (prev == v2 && curr == v1 && prev != curr) return true;
        }
        return false;
    };
    
    open.push({start, 0, 0, (double)heuristic(start, goal), 1.0});
    bestG[{start, 0}] = 0;
    
    while (!open.empty()) {
        auto [v, t, gVal, costVal, csVal] = open.top();
        open.pop();
        
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
            return path;
        }
        
        if (bestG.count({v, t}) && bestG[{v, t}] < gVal) continue;
        
        // Wait action
        if (!isBlocked(constraints, v, t + 1)) {
            int newG = gVal + 1;
            double penalty = hasConflict(v, t + 1) ? CONFLICT_PENALTY : 0;
            double newCost = newG + heuristic(v, goal) + penalty;
            double newCS = customerSatisfaction(newG, et, lt);
            
            if (!bestG.count({v, t + 1}) || newG < bestG[{v, t + 1}]) {
                bestG[{v, t + 1}] = newG;
                open.push({v, t + 1, newG, newCost, newCS});
                parent[{v, t + 1}] = {v, t};
            }
        }
        
        // Move actions
        for (int edgeIdx : g[v]) {
            auto [u1, u2] = edges[edgeIdx];
            int nv = (u1 == v) ? u2 : u1;
            
            if (isBlocked(constraints, nv, t + 1)) continue;
            if (isEdgeBlocked(constraints, v, nv, t)) continue;
            
            double penalty = 0;
            if (hasConflict(nv, t + 1)) penalty += CONFLICT_PENALTY;
            if (hasEdgeConflict(v, nv, t)) penalty += CONFLICT_PENALTY;
            
            int newG = gVal + 1;
            double newCost = newG + heuristic(nv, goal) + penalty;
            double newCS = customerSatisfaction(newG, et, lt);
            
            if (!bestG.count({nv, t + 1}) || newG < bestG[{nv, t + 1}]) {
                bestG[{nv, t + 1}] = newG;
                open.push({nv, t + 1, newG, newCost, newCS});
                parent[{nv, t + 1}] = {v, t};
            }
        }
    }
    
    return std::nullopt;
}

} // namespace Core