#pragma once

#include <vector>
#include <tuple>
#include <memory>
#include <functional>

namespace Core { 

using PythonSignals = int (*)();
// extern PythonSignals g_pythonSignals;

template<typename T>
using Ref = std::shared_ptr<T>;

template<typename U, typename ...T>
inline Ref<U> CreateRef(T&&... args) {
    return std::make_shared<U>(std::forward<T>(args)...);
}

// Constraint: vertex or edge
struct Constraint {
    int vertex, time;
    bool isEdge;
    int vertex2;
    // bool operator<(const Constraint& other) const {
    //     return std::tuple(vertex, time, isEdge, vertex2) < std::tuple(other.vertex, other.time, other.isEdge, other.vertex2);
    // }
};

using Path = std::vector<int>;
using AgentConstraints = std::vector<Constraint>;

struct Result {
    bool found;
    std::vector<Path> solution;
    double cost;
};

inline double customerSatisfaction(int arrivalTime, int et, int lt)
{
    if (arrivalTime <= et) return 1.0;
    if (arrivalTime >= lt) return 0.0;
    return (double)(lt - arrivalTime) / (lt - et);
}

inline bool isBlocked(const AgentConstraints& constraints, int vertex, int time) {
    for (const auto& c : constraints) {
        if (!c.isEdge && c.vertex == vertex && c.time == time) return true;
    }
    return false;
}

inline bool isGoalBlockedAfter(const AgentConstraints& constraints, int goal, int arrivalTime) {
    for (const auto& c : constraints) {
        if (!c.isEdge && c.vertex == goal && c.time >= arrivalTime) return true;
    }
    return false;
}

inline bool isEdgeBlocked(const AgentConstraints& constraints, int v1, int v2, int time) {
    for (const auto& c : constraints) {
        if (c.isEdge && c.time == time) {
            if ((c.vertex == v1 && c.vertex2 == v2) || (c.vertex == v2 && c.vertex2 == v1))
                return true;
        }
    }
    return false;
}

} //namespace
