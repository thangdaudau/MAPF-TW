#pragma once

#include "ICBSBase.h"
#include "CBS.h"
#include "GCBS.h"
#include <vector>
#include <tuple>
#include <optional>
#include <chrono>

namespace Core
{

class ICBS : public ICBSBase {
protected:
    // High-level
    ICBSResult highLevelSearch(bool restarted = false) override;
    void findAllConflicts(Ref<ICTNode> node);
    void processConflict(Ref<ICTNode> node, int ma1, int ma2);
    ConflictType classifyConflict(Ref<ICTNode> node, const IConflict& conflict);
    std::optional<IConflict> selectConflict(Ref<ICTNode> node);
    Ref<ICTNode> generateChild(Ref<ICTNode> parent, int metaAgent, const IConflict& conflict);
    
    // MA-CBS
    bool shouldMerge(Ref<ICTNode> node, int metaAgent1, int metaAgent2);
    Ref<ICTNode> merge(Ref<ICTNode> node, int metaAgent1, int metaAgent2);
    
    // Low-level (for meta-agent)
    std::optional<std::vector<Path>> lowLevelSearch(int metaAgent, Ref<ICTNode> node) override;
    
    // Bypass
    bool tryBypass(Ref<ICTNode> node, const IConflict& conflict);
    int countMetaAgentConflicts(Ref<ICTNode> node, int metaAgent);
    int computeTotalConflicts(Ref<ICTNode> node);
    
    // MDD for PC (with reuse capability)
    MDD buildMDD(int agent, int pathLength, const MetaAgentConstraints& constraints);
    MDD buildMDDFrom(int agent, int pathLength, 
                     const MetaAgentConstraints& constraints,
                     const MDD& parentMDD);
    int getMDDWidth(const MDD& mdd, int time);
    
    // Cost (Time Window)
    double agentCS(int agent, const Path& path);
    double metaAgentCS(const MetaAgent& ma);
    double calculateCost(Ref<ICTNode> node);
    void updateMetaAgent(Ref<ICTNode> node, int metaAgent, const std::vector<Path>& newPaths);
    
    // Utility
    std::pair<int, int> toCoord(int v);
    int heuristic(int u, int goal);
    int getPosition(const Path& path, int time);
    std::vector<Path> extractSolution(Ref<ICTNode> node);

    // Data
    int row, col;
    std::vector<std::pair<int, int>> edges;
    std::vector<std::vector<int>> g;

    int countAgent;
    std::vector<std::tuple<int, int, int, int>> agents;

    // MA-CBS
    int mergeThreshold;
    bool mergeRestartActive;
    uint32_t maxMetaAgentSize;
    std::vector<std::vector<int>> initMerges;

    GCBS llSolver;
public:
    ICBS(int row, int col, 
         std::vector<std::pair<int, int>> _edges, 
         std::vector<std::tuple<int, int, int, int>> _agents, 
         int _mergeThreshold = 25,
         bool _mergeRestartActive = true, uint32_t _maxMetaAgentSize = -1);
};

} // namespace Core