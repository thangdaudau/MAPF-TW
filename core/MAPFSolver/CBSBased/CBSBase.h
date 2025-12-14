#pragma once

#include "../base.h"
#include <string>
#include <vector>
#include <optional>
#include <cstdint>
#include <chrono>
#include <atomic>

namespace Core
{

struct CTNode {
    std::vector<AgentConstraints> constraints;
    std::vector<Path> solution;
    double cost;
    int numConflicts; // for bypassing conflict
    int totalPathLength;

    bool operator<(const CTNode& o) const {
        if (std::abs(cost - o.cost) > 1e-9) return cost < o.cost;
        if (numConflicts != o.numConflicts) return numConflicts > o.numConflicts;
        return totalPathLength > o.totalPathLength;
    }
};

struct Conflict {
    int agent1, agent2, vertex, time;
    bool isEdge;
    int vertex2;
};

using CBSResult = Result;

class CBSBase {
protected:
    virtual CBSResult highLevelSearch() = 0;
    virtual std::optional<Path> lowLevelSearch(int agent, const AgentConstraints& constraints) = 0;
    
    uint32_t time_limit_in_seconds = -1;
    std::chrono::steady_clock::time_point startTime;
    PythonSignals pythonSignals = nullptr;
    bool isTimeUp() {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - startTime).count();
        return elapsed >= time_limit_in_seconds || (pythonSignals != nullptr && pythonSignals() != 0);
    }
public:
    void setTimeLimit(uint32_t seconds) { time_limit_in_seconds = seconds; }
    void solve(PythonSignals _pythonSignals = nullptr) { pythonSignals = _pythonSignals; result = highLevelSearch(); }

    CBSResult result {false, {}, 1e9};
};

} // namespace Core