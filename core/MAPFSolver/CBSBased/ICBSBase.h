#pragma once

#include "../base.h"
#include <vector>
#include <optional>
#include <cmath>
#include <chrono>
#include <iostream>
#include <atomic>
#include <unordered_map>

namespace Core
{

using MetaAgentConstraints = AgentConstraints;

enum class ConflictType {
    NONE,
    CARDINAL,
    SEMI_CARDINAL1,
    SEMI_CARDINAL2,
    NON_CARDINAL
};

struct IConflict {
    int metaAgent1, metaAgent2;
    int vertex, time;
    bool isEdge;
    int vertex2;
    ConflictType type = ConflictType::NONE;
};

struct MDD {
    std::vector<std::vector<int>> levels;
};

struct MetaAgent {
    std::vector<int> agents;
    std::vector<Path> solution;
    double cost;
    int totalPathLength;
};

struct ConflictMatrix {
    std::vector<std::vector<std::tuple<std::optional<IConflict>, int, int>>> conflicts;
    
    void resize(int n) {
        conflicts.resize(n);
        for (int i = 0; i < n; i++) {
            conflicts[i].resize(i);
        }
    }

    void clear() {
        conflicts.clear();
    }

    void remove(int pos) {
        conflicts.erase(conflicts.begin() + pos);
        for (auto& row : conflicts) {
            if ((int)row.size() > pos) row.erase(row.begin() + pos);
        }
    }
};

// Cache MDD cho mỗi meta-agent
struct MDDCache {
    // Key: agent ID, Value: MDD
    std::unordered_map<int, MDD> mdds;
    
    void clear() {
        mdds.clear();
    }
    
    bool contains(int agent) const {
        return mdds.contains(agent);
    }
    
    const MDD& get(int agent) const {
        return mdds.at(agent);
    }
    
    void set(int agent, MDD&& mdd) {
        mdds[agent] = std::move(mdd);
    }
};

struct ICTNode {
    std::vector<MetaAgent> metaAgents;
    std::vector<MetaAgentConstraints> constraints;
    ConflictMatrix conflictMatrix;
    MDDCache mddCache;  // Cache MDD để tái sử dụng
    double cost;
    int numConflicts;
    int totalPathLength;

    bool operator<(const ICTNode& o) const {
        if (std::abs(cost - o.cost) > 1e-9) return cost < o.cost;
        if (numConflicts != o.numConflicts) return numConflicts > o.numConflicts;
        return totalPathLength > o.totalPathLength;
    }
};

using ICBSResult = Result;

class ICBSBase {
protected:
    virtual ICBSResult highLevelSearch(bool restarted = false) = 0;
    virtual std::optional<std::vector<Path>> lowLevelSearch(int metaAgent, Ref<ICTNode> node) = 0;
    
    uint32_t timeLimitSeconds = -1;
    std::chrono::steady_clock::time_point startTime;
    PythonSignals pythonSignals = nullptr;
    bool isTimeUp() {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - startTime).count();
        return elapsed >= timeLimitSeconds || (pythonSignals != nullptr && pythonSignals() != 0);
    }
    int64_t timeRemain() {
        auto now = std::chrono::steady_clock::now();
        return timeLimitSeconds - std::chrono::duration_cast<std::chrono::seconds>(now - startTime).count();
    }

public:
    void setTimeLimit(uint32_t seconds) { timeLimitSeconds = seconds; }
    void solve(PythonSignals _pythonSignals = nullptr) { pythonSignals = _pythonSignals; result = highLevelSearch(); }

    ICBSResult result{false, {}, 0};
};

} // namespace Core