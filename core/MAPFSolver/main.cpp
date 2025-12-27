#include "AStarBased/OD.h"
#include "CBSBased/CBS.h"
#include "CBSBased/ICBS.h"
#include "CBSBased/GCBS.h"
#include <vector>
#include <cassert>
#include <iostream>

int main() {
    assert(freopen("../ExampleInput/6x6x6_hard.in", "r", stdin));

    uint32_t time_limit_in_seconds;
    int n, m;
    int row, col;
    
    std::vector<std::pair<int, int>> edges;
    std::vector<std::vector<int>> g;
    
    int count_agent;
    std::vector<std::tuple<int, int, int, int>> agents;

    std::cin >> time_limit_in_seconds >> row >> col >> n >> m;
    assert(n == row * col);
    
    g.resize(n);
    for (int i = 0; i < m; i++) {
        int u, v;
        std::cin >> u >> v;
        edges.emplace_back(u, v);
        g[u].push_back(i);
        g[v].push_back(i);
    }
    
    std::cin >> count_agent;
    agents.resize(count_agent);
    for (auto& [s, goal, et, lt] : agents) {
        std::cin >> s >> goal >> et >> lt;
    }
     
    Core::GCBS solver(row, col, edges, agents);
    auto st = clock();
    solver.solve();
    std::cerr << (clock() - st) / double(CLOCKS_PER_SEC) << '\n';

    auto result = solver.result;
    
    if (result.found) {
        std::cout << "SOLUTION FOUND\n";
        for (int i = 0; i < count_agent; i++) {
            std::cout << result.solution[i].size();
            for (int v : result.solution[i]) std::cout << " " << v;
            std::cout << "\n";
        }
        std::cout << "Average_CS: " << result.cost / count_agent << "\n";
    } else {
        std::cout << "NO_SOLUTION\n";
    }
    
    return 0;
}