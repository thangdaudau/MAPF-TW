#include "CBS.h"
#include <queue>
#include <map>
#include <chrono>
#include <iostream>
#include <iomanip>
#include <set>
#include <cassert>
#include <ranges>
#include <algorithm>

namespace Core {

// high-level section
CBSResult CBS::highLevelSearch()
{
    startTime = std::chrono::steady_clock::now();
    
    auto cmp = [](Ref<CTNode> a, Ref<CTNode> b) {
        return *a < *b;
    };
    std::priority_queue<Ref<CTNode>, std::vector<Ref<CTNode>>, decltype(cmp)> open(cmp);
    
    Ref<CTNode> root = CreateRef<CTNode>();
    root->constraints.resize(count_agent);
    root->solution.resize(count_agent);
    for (int i = 0; i < count_agent; i++) {
        auto path = lowLevelSearch(i, root->constraints[i]);
        if (!path) return {false, {}, 0};
        root->solution[i] = *path;
    }
    root->cost = calculateCost(root->solution);
    root->numConflicts = computeTotalConflicts(root->solution);
    root->totalPathLength = 0;
    for (const auto& p : root->solution) {
        root->totalPathLength += p.size();
    }

    open.push(root);

    while (!open.empty()) {
        if (isTimeUp()) {
            return {false, {}, 0};
        }
        
        Ref<CTNode> cur = open.top();
        open.pop();

        auto conflict = findFirstConflict(cur->solution);

        if (!conflict) {
            return {true, cur->solution, cur->cost};
        }
        
        if (tryBypass(cur, *conflict)) {
            open.push(cur);
            continue;
        }
        
        for (int ai : {conflict->agent1, conflict->agent2}) {
            auto child = generateChild(cur, ai, *conflict);
            if (child) {
                open.push(child);
            }
        }
    }

    return {false, {}, 0};
}

std::optional<Conflict> CBS::findFirstConflict(const std::vector<Path> &solution)
{
    int maxLen = 0;
    for (const auto& p : solution) maxLen = std::max(maxLen, (int)p.size());
    
    auto getPos = [&](int agent, int time) -> int {
        if (time < (int)solution[agent].size()) return solution[agent][time];
        return solution[agent].back();
    };
    
    for (int t = 0; t < maxLen; t++) {
        for (int i = 0; i < count_agent; i++) {
            for (int j = i + 1; j < count_agent; j++) {
                if (getPos(i, t) == getPos(j, t)) {
                    return Conflict{i, j, getPos(i, t), t, false, -1};
                }
            }
        }
        
        if (t > 0) {
            for (int i = 0; i < count_agent; i++) {
                for (int j = i + 1; j < count_agent; j++) {
                    int pi0 = getPos(i, t-1), pi1 = getPos(i, t);
                    int pj0 = getPos(j, t-1), pj1 = getPos(j, t);
                    if (pi0 == pj1 && pi1 == pj0 && pi0 != pi1) {
                        return Conflict{i, j, pi0, t-1, true, pi1};
                    }
                }
            }
        }
    }
    return std::nullopt;
}

Ref<CTNode> CBS::generateChild(Ref<CTNode> parent, int agent, const Conflict &conflict)
{
    Ref<CTNode> child = CreateRef<CTNode>(*parent);
    
    child->constraints[agent].push_back({conflict.vertex, conflict.time, 
                                         conflict.isEdge, conflict.vertex2});
    
    auto newPath = lowLevelSearch(agent, child->constraints[agent]);
    if (!newPath) return nullptr;
    
    updateAgentPath(child, agent, *newPath);
    
    return child;
}

double CBS::agentCS(int agent, const Path &path)
{
    auto [s, goal, et, lt] = agents[agent];
    return customerSatisfaction(path.size() - 1, et, lt);
}

double CBS::calculateCost(const std::vector<Path> &solution)
{
    double totalCS = 0.0;
    for (int i = 0; i < count_agent; i++) {
        totalCS += agentCS(i, solution[i]);
    }
    return totalCS;
}

void CBS::updateAgentPath(Ref<CTNode> node, int agent, const Path &newPath)
{
    // Update cost incrementally
    double oldCS = agentCS(agent, node->solution[agent]);
    double newCS = agentCS(agent, newPath);
    node->cost += (newCS - oldCS);

    node->totalPathLength += (int)newPath.size() - (int)node->solution[agent].size();
    
    // Update numConflicts incrementally
    int oldConflicts = countAgentConflicts(node->solution, agent);
    node->solution[agent] = newPath;
    int newConflicts = countAgentConflicts(node->solution, agent);
    node->numConflicts += (newConflicts - oldConflicts);
}




// low-level section
std::optional<Path> CBS::lowLevelSearch(int agent, const AgentConstraints &constraints)
{
    auto [start, goal, et, lt] = agents[agent];
    
    std::priority_queue<AStarState, std::vector<AStarState>, std::greater<>> open;
    std::map<std::pair<int,int>, int> bestG;
    std::map<std::pair<int,int>, std::pair<int,int>> parent;
    
    open.push({start, 0, 0, heuristic(start, goal)});
    bestG[{start, 0}] = 0;
    
    while (!open.empty()) {
        auto [v, t, gVal, fVal] = open.top();
        open.pop();
        
        if (v == goal && !isGoalBlockedAfter(constraints, goal, t) && !isGoalBlockedAfter(extraConstraints, goal, t)) {
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
        if (!isBlocked(constraints, v, t + 1) && !isBlocked(extraConstraints, v, t + 1)) {
            int newG = gVal + 1;
            if (!bestG.count({v, t + 1}) || newG < bestG[{v, t + 1}]) {
                bestG[{v, t + 1}] = newG;
                open.push({v, t + 1, newG, newG + heuristic(v, goal)});
                parent[{v, t + 1}] = {v, t};
            }
        }
        
        // Move actions
        for (int edgeIdx : g[v]) {
            auto [u1, u2] = edges[edgeIdx];
            int nv = (u1 == v) ? u2 : u1;
            
            if (isBlocked(constraints, nv, t + 1) || isBlocked(extraConstraints, nv, t + 1)) continue;
            if (isEdgeBlocked(constraints, v, nv, t) || isEdgeBlocked(extraConstraints, v, nv, t)) continue;
            
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

std::pair<int, int> CBS::toCoord(int v)
{
    return {v / col, v % col};
}

int CBS::heuristic(int u, int goal)
{
    auto [ux, uy] = toCoord(u);
    auto [gx, gy] = toCoord(goal);
    return std::abs(ux - gx) + std::abs(uy - gy);
}


// bypass section
bool CBS::tryBypass(Ref<CTNode> node, const Conflict &conflict)
{
    int originalNC = node->numConflicts;
    for (int ai : {conflict.agent1, conflict.agent2}) {
        AgentConstraints tempConstraints = node->constraints[ai];
        tempConstraints.push_back({conflict.vertex, conflict.time, 
                                   conflict.isEdge, conflict.vertex2});
        
        auto newPath = lowLevelSearch(ai, tempConstraints);
        if (!newPath) continue;
        
        // Check same cost for this agent only
        double oldAgentCS = agentCS(ai, node->solution[ai]);
        double newAgentCS = agentCS(ai, *newPath);
        
        if (std::abs(newAgentCS - oldAgentCS) < 1e-9) {
            Path savedPath = node->solution[ai];
            updateAgentPath(node, ai, *newPath);
            
            if (node->numConflicts < originalNC) {
                return true;
            }
            
            // Revert
            updateAgentPath(node, ai, savedPath);
        }
    }
    return false;
}

int CBS::countAgentConflicts(const std::vector<Path> &solution, int agent)
{
    int maxLen = 0;
    for (const auto& p : solution) maxLen = std::max(maxLen, (int)p.size());
    
    auto getPos = [&](int a, int t) -> int {
        if (t < (int)solution[a].size()) return solution[a][t];
        return solution[a].back();
    };
    
    int count = 0;
    for (int t = 0; t < maxLen; t++) {
        for (int j = 0; j < count_agent; j++) {
            if (j == agent) continue;
            if (getPos(agent, t) == getPos(j, t)) count++;
        }
        
        if (t > 0) {
            for (int j = 0; j < count_agent; j++) {
                if (j == agent) continue;
                int pa0 = getPos(agent, t-1), pa1 = getPos(agent, t);
                int pj0 = getPos(j, t-1), pj1 = getPos(j, t);
                if (pa0 == pj1 && pa1 == pj0 && pa0 != pa1) count++;
            }
        }
    }
    return count;
}

int CBS::computeTotalConflicts(const std::vector<Path> &solution)
{
    int maxLen = 0;
    for (const auto& p : solution) maxLen = std::max(maxLen, (int)p.size());
    
    auto getPos = [&](int agent, int time) -> int {
        if (time < (int)solution[agent].size()) return solution[agent][time];
        return solution[agent].back();
    };
    
    int count = 0;
    for (int t = 0; t < maxLen; t++) {
        for (int i = 0; i < count_agent; i++) {
            for (int j = i + 1; j < count_agent; j++) {
                if (getPos(i, t) == getPos(j, t)) count++;
            }
        }
        if (t > 0) {
            for (int i = 0; i < count_agent; i++) {
                for (int j = i + 1; j < count_agent; j++) {
                    int pi0 = getPos(i, t-1), pi1 = getPos(i, t);
                    int pj0 = getPos(j, t-1), pj1 = getPos(j, t);
                    if (pi0 == pj1 && pi1 == pj0 && pi0 != pi1) count++;
                }
            }
        }
    }
    return count;
}

CBS::CBS(int _row, int _col, std::vector<std::pair<int, int>> _edges, std::vector<std::tuple<int, int, int, int>> _agents)
: row(_row), col(_col), edges(std::move(_edges)), agents(std::move(_agents))
{
    g.resize(row * col);
    for (int id = 0; id < int(edges.size()); id++) {
        auto [u, v] = edges[id];
        g[u].push_back(id);
        g[v].push_back(id);
    }
    count_agent = static_cast<int>(agents.size());
}

void CBS::setAgents(std::vector<std::tuple<int, int, int, int>> _agents)
{
    agents = std::move(_agents);
    count_agent = (int)agents.size();
}

void CBS::setExtraConstraints(AgentConstraints _extraConstraints)
{
    extraConstraints = std::move(_extraConstraints);
}

} // namespace Core