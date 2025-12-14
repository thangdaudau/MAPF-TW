#pragma once

#include <vector>
#include <cstring>
#include <cstdlib>
#include <ctime>
#include <algorithm>
#include <cmath>
#include <queue>

namespace Core {

/**
 * Minimum Vertex Cover using NuMVC local search algorithm (TSEWF)
 * Based on the implementation by Shaowei Cai
 * Input: adjacency list g where g[i] contains neighbors of vertex i
 * Output: vector of vertices in the minimum vertex cover
 */
inline std::vector<int> MinimumVertexCover(const std::vector<std::vector<int>>& g) {
    int v_num = g.size();
    if (v_num == 0) return {};
    
    // Build edge list from adjacency list
    struct Edge {
        int v1;
        int v2;
    };
    
    std::vector<Edge> edge;
    std::vector<std::vector<int>> v_edges(v_num);
    std::vector<std::vector<int>> v_adj(v_num);
    std::vector<int> v_edge_count(v_num, 0);
    
    // Count edges and build structures
    for (int i = 0; i < v_num; i++) {
        for (int j : g[i]) {
            if (i < j) {  // Avoid duplicate edges
                edge.push_back({i, j});
            }
        }
    }
    
    int e_num = edge.size();
    if (e_num == 0) return {};
    
    // Build v_adj and v_edges
    for (int i = 0; i < v_num; i++) {
        v_edge_count[i] = 0;
    }
    
    for (int e = 0; e < e_num; e++) {
        v_edge_count[edge[e].v1]++;
        v_edge_count[edge[e].v2]++;
    }
    
    for (int v = 0; v < v_num; v++) {
        v_adj[v].resize(v_edge_count[v]);
        v_edges[v].resize(v_edge_count[v]);
    }
    
    std::vector<int> v_edge_count_tmp(v_num, 0);
    for (int e = 0; e < e_num; e++) {
        int v1 = edge[e].v1;
        int v2 = edge[e].v2;
        
        v_edges[v1][v_edge_count_tmp[v1]] = e;
        v_edges[v2][v_edge_count_tmp[v2]] = e;
        
        v_adj[v1][v_edge_count_tmp[v1]] = v2;
        v_adj[v2][v_edge_count_tmp[v2]] = v1;
        
        v_edge_count_tmp[v1]++;
        v_edge_count_tmp[v2]++;
    }
    
    // Algorithm parameters
    const long long max_steps = 100;
    [[maybe_unused]] const int try_step = 100;
    const float p_scale = 0.3f;
    const int threshold = (int)(0.5 * v_num);
    const int optimal_size = 0;  // Run until step limit
    
    // Data structures
    std::vector<int> edge_weight(e_num, 1);
    std::vector<int> dscore(v_num, 0);
    std::vector<long long> time_stamp(v_num, 0);
    std::vector<int> v_in_c(v_num, 0);
    std::vector<int> conf_change(v_num, 1);
    
    // Uncovered edge stack
    std::vector<int> uncov_stack;
    uncov_stack.reserve(e_num);
    std::vector<int> index_in_uncov_stack(e_num);
    
    // Remove candidates
    std::vector<int> remove_cand(v_num);
    std::vector<int> index_in_remove_cand(v_num, 0);
    int remove_cand_size = 0;
    
    int ave_weight = 1;
    int delta_total_weight = 0;
    int c_size = 0;
    int best_c_size = v_num;
    std::vector<int> best_v_in_c(v_num, 0);
    int tabu_remove = 0;
    int best_cov_v = 0;
    
    // Lambda functions
    auto uncover = [&](int e) {
        index_in_uncov_stack[e] = uncov_stack.size();
        uncov_stack.push_back(e);
    };
    
    auto cover = [&](int e) {
        int last_uncov_edge = uncov_stack.back();
        uncov_stack.pop_back();
        int index = index_in_uncov_stack[e];
        uncov_stack[index] = last_uncov_edge;
        index_in_uncov_stack[last_uncov_edge] = index;
    };
    
    auto add = [&](int v) {
        v_in_c[v] = 1;
        dscore[v] = -dscore[v];
        
        int edge_count = v_edge_count[v];
        for (int i = 0; i < edge_count; i++) {
            int e = v_edges[v][i];
            int n = v_adj[v][i];
            
            if (v_in_c[n] == 0) {
                dscore[n] -= edge_weight[e];
                conf_change[n] = 1;
                cover(e);
            } else {
                dscore[n] += edge_weight[e];
            }
        }
    };
    
    auto remove = [&](int v) {
        v_in_c[v] = 0;
        dscore[v] = -dscore[v];
        conf_change[v] = 0;
        
        int edge_count = v_edge_count[v];
        for (int i = 0; i < edge_count; i++) {
            int e = v_edges[v][i];
            int n = v_adj[v][i];
            
            if (v_in_c[n] == 0) {
                dscore[n] += edge_weight[e];
                conf_change[n] = 1;
                uncover(e);
            } else {
                dscore[n] -= edge_weight[e];
            }
        }
    };
    
    auto update_best_sol = [&]() {
        for (int i = 0; i < v_num; i++) {
            best_v_in_c[i] = v_in_c[i];
        }
        best_c_size = c_size;
    };
    
    auto reset_remove_cand = [&]() {
        int j = 0;
        for (int v = 0; v < v_num; v++) {
            if (v_in_c[v] == 1) {
                remove_cand[j] = v;
                index_in_remove_cand[v] = j;
                j++;
            } else {
                index_in_remove_cand[v] = 0;
            }
        }
        remove_cand_size = j;
    };
    
    auto update_best_cov_v = [&]() {
        best_cov_v = remove_cand[0];
        for (int i = 1; i < remove_cand_size; i++) {
            int v = remove_cand[i];
            if (v == tabu_remove) continue;
            
            if (dscore[v] < dscore[best_cov_v])
                continue;
            else if (dscore[v] > dscore[best_cov_v])
                best_cov_v = v;
            else if (time_stamp[v] < time_stamp[best_cov_v])
                best_cov_v = v;
        }
    };
    
    auto forget_edge_weights = [&]() {
        for (int v = 0; v < v_num; v++)
            dscore[v] = 0;
        
        int new_total_weight = 0;
        for (int e = 0; e < e_num; e++) {
            edge_weight[e] = edge_weight[e] * p_scale;
            new_total_weight += edge_weight[e];
            
            if (v_in_c[edge[e].v1] + v_in_c[edge[e].v2] == 0) {
                dscore[edge[e].v1] += edge_weight[e];
                dscore[edge[e].v2] += edge_weight[e];
            } else if (v_in_c[edge[e].v1] + v_in_c[edge[e].v2] == 1) {
                if (v_in_c[edge[e].v1] == 1)
                    dscore[edge[e].v1] -= edge_weight[e];
                else
                    dscore[edge[e].v2] -= edge_weight[e];
            }
        }
        ave_weight = new_total_weight / e_num;
    };
    
    auto update_edge_weight = [&]() {
        for (int i = 0; i < (int)uncov_stack.size(); i++) {
            int e = uncov_stack[i];
            edge_weight[e] += 1;
            dscore[edge[e].v1] += 1;
            dscore[edge[e].v2] += 1;
        }
        
        delta_total_weight += uncov_stack.size();
        if (delta_total_weight >= e_num) {
            ave_weight += 1;
            delta_total_weight -= e_num;
        }
        
        if (ave_weight >= threshold) {
            forget_edge_weights();
        }
    };
    
    auto update_target_size = [&]() {
        c_size--;
        
        int max_improvement = -100000000;
        int max_vertex = 0;
        
        for (int v = 0; v < v_num; v++) {
            if (v_in_c[v] == 0) continue;
            if (dscore[v] > max_improvement) {
                max_improvement = dscore[v];
                max_vertex = v;
            }
        }
        remove(max_vertex);
        reset_remove_cand();
    };
    
    // Initialize solution
    for (int v = 0; v < v_num; v++) {
        v_in_c[v] = 0;
        dscore[v] = 0;
        conf_change[v] = 1;
        time_stamp[v] = 0;
    }
    
    for (int e = 0; e < e_num; e++) {
        edge_weight[e] = 1;
        dscore[edge[e].v1] += edge_weight[e];
        dscore[edge[e].v2] += edge_weight[e];
    }
    
    for (int e = 0; e < e_num; e++) {
        uncover(e);
    }
    
    // Greedy initial solution
    while (!uncov_stack.empty()) {
        int best_vertex_improvement = 0;
        int best_count = 0;
        std::vector<int> best_array(v_num);
        
        for (int v = 0; v < v_num; v++) {
            if (v_in_c[v] == 1) continue;
            
            if (dscore[v] > best_vertex_improvement) {
                best_vertex_improvement = dscore[v];
                best_array[0] = v;
                best_count = 1;
            } else if (dscore[v] == best_vertex_improvement) {
                best_array[best_count] = v;
                best_count++;
            }
        }
        
        if (best_count > 0) {
            add(best_array[rand() % best_count]);
            c_size++;
        }
    }
    
    update_best_sol();
    reset_remove_cand();
    update_best_cov_v();
    
    // Local search
    long long step = 1;
    while (step <= max_steps) {
        if (uncov_stack.empty()) {
            update_best_sol();
            
            if (c_size == optimal_size)
                break;
            
            update_target_size();
            continue;
        }
        
        update_best_cov_v();
        remove(best_cov_v);
        
        int e = uncov_stack[rand() % uncov_stack.size()];
        int v1 = edge[e].v1;
        int v2 = edge[e].v2;
        
        int best_add_v;
        if (conf_change[v1] == 0)
            best_add_v = v2;
        else if (conf_change[v2] == 0)
            best_add_v = v1;
        else {
            if (dscore[v1] > dscore[v2] || (dscore[v1] == dscore[v2] && time_stamp[v1] < time_stamp[v2]))
                best_add_v = v1;
            else
                best_add_v = v2;
        }
        
        add(best_add_v);
        
        int index = index_in_remove_cand[best_cov_v];
        index_in_remove_cand[best_cov_v] = 0;
        
        remove_cand[index] = best_add_v;
        index_in_remove_cand[best_add_v] = index;
        
        time_stamp[best_add_v] = time_stamp[best_cov_v] = step;
        
        tabu_remove = best_add_v;
        
        update_edge_weight();
        
        step++;
    }
    
    // Return result
    std::vector<int> result;
    for (int v = 0; v < v_num; v++) {
        if (best_v_in_c[v] == 1) {
            result.push_back(v);
        }
    }
    
    return result;
}


inline std::vector<std::vector<int>> computeHeuristicTable(
    int row,
    int col,
    const std::vector<std::vector<int>>& g,
    const std::vector<std::pair<int, int>>& edges,
    const std::vector<std::tuple<int, int, int, int>>& agents
) {
    std::vector heuristicTable(agents.size(), std::vector(row * col, 1000000)); // 1e6 as infinity
    for (int i = 0; i < (int)agents.size(); ++i) {
        auto [start, goal, et, lt] = agents[i];
        
        std::queue<std::pair<int, int>> q;
        q.push({goal, 0});
        heuristicTable[i][goal] = 0;
        
        while (!q.empty()) {
            auto [u, dist] = q.front();
            q.pop();
            
            for (int id : g[u]) {
                int v = u ^ edges[id].first ^ edges[id].second;
                
                if (heuristicTable[i][v] > dist + 1) {
                    heuristicTable[i][v] = dist + 1;
                    q.push({v, dist + 1});
                }
            }
        }
    }
    return heuristicTable;
}

} // namespace Core