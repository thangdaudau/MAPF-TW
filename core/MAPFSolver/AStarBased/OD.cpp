#include "OD.h"
#include "../utils/GraphAlgos.h"
#include <queue>
#include <set>
#include <iostream>
#include <unordered_set>
#include <boost/functional/hash.hpp>

namespace Core {

OD::OD(int _row, int _col, std::vector<std::pair<int, int>> _edges, std::vector<std::tuple<int, int, int, int>> _agents)
    : row(_row), col(_col), edges(std::move(_edges)), agents(std::move(_agents)) {
    g.resize(row * col);
    for (int id = 0; id < int(edges.size()); id++) {
        auto [u, v] = edges[id];
        g[u].push_back(id);
        g[v].push_back(id);
    }

    countAgent = static_cast<int>(agents.size());
    heuristicTable = computeHeuristicTable(row, col, g, edges, agents);
}

bool OD::isTimeUp() {
    if (timeLimitSeconds == static_cast<uint32_t>(-1)) return false;
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - startTime).count();
    return elapsed >= timeLimitSeconds || (pythonSignals != nullptr && pythonSignals() != 0);
}

bool OD::isValidMove(int agent, int from, int to, int timestep, Ref<ODNode> node) {
    for (int a = 0; a < node->assignedCount; ++a) {
        int otherPos = getPos(node->solution[a], timestep);
        int otherPrevPos = getPos(node->solution[a], timestep - 1);
        
        if (otherPos == to) return false;
        if (otherPrevPos == to && otherPos == from) return false;
    }
    
    return true;
}

void OD::updateAgent(Ref<ODNode> node, int agent, int to) {
    auto [start, goal, et, lt] = agents[agent];

    int prePos = node->solution[agent].back();
    int preTime = static_cast<int>(node->solution[agent].size()) - 1;
    int preDist = heuristicTable[agent][prePos];
    int preEstimateArrival = preTime + preDist;
    node->cost -= customerSatisfaction(preEstimateArrival, et, lt);
    node->totalPathLength -= static_cast<int>(node->solution[agent].size()) - 1 + preDist;

    if (node->solution[agent].back() != to) {
        node->solution[agent].resize(node->timeStep + 1, prePos);
        node->solution[agent].push_back(to);
    }
    node->assignedCount++;

    int currentPos = node->solution[agent].back();
    int currentTime = static_cast<int>(node->solution[agent].size()) - 1;
    int dist = heuristicTable[agent][currentPos];
    int estimatedArrival = currentTime + dist;
    node->cost += customerSatisfaction(estimatedArrival, et, lt);
    node->totalPathLength += static_cast<int>(node->solution[agent].size()) - 1 + dist;
}

int OD::getPos(const Path &path, int pos)
{
    return pos < (int)path.size() ? path[pos] : path.back();
}

void OD::solve(PythonSignals _pythonSignals) {
    startTime = std::chrono::steady_clock::now();
    pythonSignals = _pythonSignals;
    result = {false, {}, 0};
    
    if (countAgent == 0) {
        result = {true, {}, 0};
        return;
    }
        
    Ref<ODNode> root = CreateRef<ODNode>();
    root->solution.resize(countAgent);
    root->assignedCount = 0;
    root->timeStep = 0;
    
    for (int i = 0; i < countAgent; ++i) {
        int start = std::get<0>(agents[i]);
        root->solution[i].push_back(start);
    }
    
    root->cost = 0.0;
    root->totalPathLength = 0;
    for (int i = 0; i < countAgent; ++i) {
        auto [start, goal, et, lt] = agents[i];
        int dist = heuristicTable[i][start];
        int estimatedArrival = dist;
        root->cost += customerSatisfaction(estimatedArrival, et, lt);
        root->totalPathLength += dist;
    }

    auto cmp = [](Ref<ODNode> a, Ref<ODNode> b) {
        return *a < *b;
    };
    
    std::priority_queue<Ref<ODNode>, std::vector<Ref<ODNode>>, decltype(cmp)> open(cmp);
    open.push(root);
    std::unordered_set<std::vector<int>, decltype([](const auto& v) {
        return boost::hash_range(v.begin(), v.end());
    })> closed;
    
    while (!open.empty()) {
        if (isTimeUp()) break;
        
        Ref<ODNode> curr = open.top();
        open.pop();

        // STANDARD STATE
        if (curr->assignedCount == countAgent) {
            bool allAtGoal = true;
            for (int i = 0; i < countAgent; ++i) {
                int goal = std::get<1>(agents[i]);
                int currentPos = curr->solution[i].back();
                // int currentTime = static_cast<int>(curr->solution[i].size()) - 1;
                
                if (currentPos != goal) {
                    allAtGoal = false;
                    break;
                }
            }
            
            if (allAtGoal) {
                result.found = true;
                result.solution = curr->solution;
                result.cost = curr->cost;
                return;
            }
            
            // Create new node for next timestep
            curr->timeStep++;
            curr->assignedCount = 0;

            // CHECK DUPLICATE
            std::vector<int> currentConfig;
            currentConfig.reserve(countAgent);
            for(const auto& p : curr->solution) {
                currentConfig.push_back(p.back());
            }

            if (closed.insert(currentConfig).second == false) {
                continue;
            }
        }
        
        // INTERMEDIATE STATE
        int agentIdx = curr->assignedCount;
        int currPos = curr->solution[agentIdx].back();
        int nextTime = curr->timeStep + 1;
        
        // Move
        for (int id : g[currPos]) {
            int nextPos = currPos ^ edges[id].first ^ edges[id].second;
            if (isValidMove(agentIdx, currPos, nextPos, nextTime, curr)) {
                Ref<ODNode> child = CreateRef<ODNode>(*curr);
                updateAgent(child, agentIdx, nextPos);
                open.push(child);
            }
        }
        
        // Wait
        if (isValidMove(agentIdx, currPos, currPos, nextTime, curr)) {
            updateAgent(curr, agentIdx, currPos);
            open.push(curr);
        }
    }
}

} // namespace Core