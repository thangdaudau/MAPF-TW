#pragma once

#include "CBS.h"
#include "../utils/GraphAlgos.h"
#include <queue>
#include <vector>

namespace Core {

// GCBS modes
enum class GCBSMode {
    GCBS_H,   // Greedy high-level only
    GCBS_L,   // Greedy low-level only  
    GCBS_LH   // Greedy both levels (default)
};

// Conflict heuristic types
enum class ConflictHeuristic {
    NUM_CONFLICTS,           // h1: Total number of conflicts
    NUM_CONFLICTING_AGENTS,  // h2: Number of agents with conflicts
    NUM_CONFLICTING_PAIRS,   // h3: Number of agent pairs with conflicts (recommended)
    VERTEX_COVER,            // h4: Vertex cover of conflict graph
    ALTERNATING              // h5: Alternate between heuristics
};

// Extended CT Node with heuristic information
struct CTNodeH {
    int id;
    Ref<CTNode> base;
    
    // Conflict Analysis Table (CAT)
    std::vector<std::vector<bool>> conflictMatrix;
    std::vector<std::vector<int>> conflictCount;
    
    // Cached heuristic values
    int h1, h2, h3;
    
    CTNodeH() : id(0), base(nullptr), h1(0), h2(0), h3(0) {}
    
    inline void init(int numAgents) {
        conflictMatrix.assign(numAgents, std::vector<bool>(numAgents, false));
        conflictCount.assign(numAgents, std::vector<int>(numAgents, 0));
    }
    
    inline void computeHeuristics(int numAgents) {
        h1 = 0;
        h2 = 0;
        h3 = 0;
        
        std::vector<bool> hasConflict(numAgents, false);
        
        for (int i = 0; i < numAgents; i++) {
            for (int j = i + 1; j < numAgents; j++) {
                if (conflictMatrix[i][j]) {
                    h1 += conflictCount[i][j];
                    h3++;
                    hasConflict[i] = true;
                    hasConflict[j] = true;
                }
            }
        }
        
        for (int i = 0; i < numAgents; i++) {
            if (hasConflict[i]) h2++;
        }
    }
};

class GCBS : public CBS {
private:
    GCBSMode mode;
    ConflictHeuristic heuristicType;
    int alternatingCounter;
    
    CBSResult highLevelSearch() override;
    
    std::optional<Path> lowLevelSearchWithConflictAvoidance(
        int agent, 
        const AgentConstraints& constraints,
        const std::vector<Path>& currentSolution
    );
    
    void updateAgentPathH(Ref<CTNodeH> node, int agent, const Path& newPath);
    
    Ref<CTNodeH> buildCTNodeH(Ref<CTNode> ctNode);
    
    int computeVertexCover(Ref<CTNodeH> node);

public:
    GCBS(int _row, int _col, 
         std::vector<std::pair<int, int>> _edges, 
         std::vector<std::tuple<int, int, int, int>> _agents,
         GCBSMode _mode = GCBSMode::GCBS_LH,
         ConflictHeuristic _heuristicType = ConflictHeuristic::NUM_CONFLICTING_PAIRS);
    
    void setMode(GCBSMode _mode) { mode = _mode; }
    void setHeuristicType(ConflictHeuristic _heuristicType) { heuristicType = _heuristicType; }
};

} // namespace Core