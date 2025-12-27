#pragma once

#include "../base.h"
#include <vector>
#include <tuple>
#include <chrono>
#include <memory>

namespace Core {

struct ODNode {
    std::vector<Path> solution;
    int assignedCount;
    int timeStep;
    double cost;
    int totalPathLength;
    
    bool operator<(const ODNode& o) const {
        if (std::abs(cost - o.cost) > 1e-9) return cost < o.cost;
        return totalPathLength > o.totalPathLength;
    }
};

class OD {
protected:
    uint32_t timeLimitSeconds = -1;
    std::chrono::steady_clock::time_point startTime;
    PythonSignals pythonSignals = nullptr;
    
    int row, col;
    std::vector<std::pair<int, int>> edges;
    std::vector<std::vector<int>> g;
    
    int countAgent;
    std::vector<std::tuple<int, int, int, int>> agents;

    std::vector<std::vector<int>> heuristicTable;
    
    // Helper functions
    bool isValidMove(int agent, int from, int to, int timestep, Ref<ODNode> node);
    bool isTimeUp();
    void updateAgent(Ref<ODNode> node, int agent, int to);
    int getPos(const Path& path, int pos);
    
public:
    Result result{false, {}, 0};
    
    OD(int _row, int _col, std::vector<std::pair<int, int>> _edges, std::vector<std::tuple<int, int, int, int>> _agents);
    
    void setTimeLimit(uint32_t seconds) { timeLimitSeconds = seconds; }
    void solve(PythonSignals _pythonSignals = nullptr);
};

} // namespace Core