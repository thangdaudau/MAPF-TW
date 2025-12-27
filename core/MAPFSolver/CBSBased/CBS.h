#pragma once

#include "CBSBase.h"
#include <tuple>

namespace Core
{

struct AStarState {
    int vertex, time, g, f;
    
    bool operator>(const AStarState& o) const {
        return f > o.f;
    }
};

class CBS: public CBSBase {
protected:
    // high-level
    CBSResult highLevelSearch() override;
    std::optional<Conflict> findFirstConflict(const std::vector<Path>& solution);
    Ref<CTNode> generateChild(Ref<CTNode> parent, int agent, const Conflict& conflict);
    double agentCS(int agent, const Path& path);
    double calculateCost(const std::vector<Path>& solution);
    void updateAgentPath(Ref<CTNode> node, int agent, const Path& newPath);
    
    // low-level
    std::optional<Path> lowLevelSearch(int agent, const AgentConstraints& constraints) override;
    std::pair<int,int> toCoord(int v);
    int heuristic(int u, int goal);

    // bypass
    bool tryBypass(Ref<CTNode> node, const Conflict& conflict);
    int countAgentConflicts(const std::vector<Path>& solution, int agent);
    int computeTotalConflicts(const std::vector<Path>& solution);

    int row, col;
    std::vector<std::pair<int, int>> edges;
    std::vector<std::vector<int>> g;

    int count_agent;
    std::vector<std::tuple<int, int, int, int>> agents;

    AgentConstraints extraConstraints;

public:
    CBS(int _row, int _col, std::vector<std::pair<int, int>> _edges, std::vector<std::tuple<int, int, int, int>> _agents);
    void setAgents(std::vector<std::tuple<int, int, int, int>> _agents);
    void setExtraConstraints(AgentConstraints _extraConstraints);
};
    
} // namespace Core