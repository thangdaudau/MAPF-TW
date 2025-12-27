import pulp


def MIP_MAPF_TW(V: int, E: list[(int, int)], S: list[(int, int, int, int)], T: int, time_limit_in_seconds: int):
    """
    Solve MAPF-TW using Mixed Integer Linear Programming with PuLP.
    Given an undirected graph G(V, E).
    
    Args:
        V: Number of vertices in the graph
        E: List of undirected edges (u, v)
        S: List of agent specifications (start, goal, et, lt) for each agent
        T: Time horizon
        time_limit_in_seconds: Time limit for solver (-1 for no limit)
    
    Returns:
        Solution with paths and customer satisfaction score
    """
    # Create problem
    prob = pulp.LpProblem("MAPF_TW", pulp.LpMaximize)

    M = 1024  # Big M constant
    n = len(S)  # Number of agents
    
    # Build time-expanded network
    time_expanded_edges = []
    edge_to_idx = {}

    for t in range(T - 1):
        for u, v in E:
            # Forward direction: u^t -> v^{t+1}
            idx = len(time_expanded_edges)
            time_expanded_edges.append((u, t, v, t + 1))
            edge_to_idx[(u, t, v, t + 1)] = idx
            
            # Backward direction: v^t -> u^{t+1}
            idx = len(time_expanded_edges)
            time_expanded_edges.append((v, t, u, t + 1))
            edge_to_idx[(v, t, u, t + 1)] = idx
        
        # Add self-loops: v^t -> v^{t+1} for waiting
        for v in range(V):
            idx = len(time_expanded_edges)
            time_expanded_edges.append((v, t, v, t + 1))
            edge_to_idx[(v, t, v, t + 1)] = idx
    
    num_edges = len(time_expanded_edges)
    
    # Variables: x[i][j] = 1 if agent i uses edge e_j
    x = pulp.LpVariable.dicts("x", 
                              ((i, j) for i in range(n) for j in range(num_edges)),
                              cat='Binary')
    
    # Variables: x_arrival[i][t] (agent i arrives at goal at time t)
    x_arrival = pulp.LpVariable.dicts("x_arrival",
                                      ((i, t) for i in range(n) for t in range(T)),
                                      cat='Binary')
    
    # Variables: y[i][t] = 1 if agent i has reached goal by time t
    y = pulp.LpVariable.dicts("y",
                              ((i, t) for i in range(n) for t in range(T)),
                              cat='Binary')
    
    # Customer satisfaction variables
    cs = pulp.LpVariable.dicts("cs",
                               range(n),
                               lowBound=0,
                               upBound=1,
                               cat='Continuous')
    
    # Binary variables for piecewise linear constraints
    b = pulp.LpVariable.dicts("b",
                              ((i, j) for i in range(n) for j in range(3)),
                              cat='Binary')
    
    # Build adjacency lists for time-expanded network
    delta_plus = {}   # Incoming edges to v^t
    delta_minus = {}  # Outgoing edges from v^t
    
    for v in range(V):
        for t in range(T):
            delta_plus[(v, t)] = []
            delta_minus[(v, t)] = []
    
    for j, (u, t_u, v, t_v) in enumerate(time_expanded_edges):
        delta_minus[(u, t_u)].append(j)
        delta_plus[(v, t_v)].append(j)
    
    # Constraint (1): Edge capacity - at most one agent per edge
    for j in range(num_edges):
        prob += pulp.lpSum(x[i, j] for i in range(n)) <= 1, f"edge_capacity_{j}"
    
    # Terminal vertices
    terminal_vertices = set()
    for s, g, _, _ in S:
        terminal_vertices.add((s, 0))
        terminal_vertices.add((g, T - 1))
    
    # Constraint (2): Flow conservation at non-terminal vertices
    for i in range(n):
        s_i, g_i, et_i, lt_i = S[i]
        for v in range(V):
            for t in range(T):
                if (v, t) not in terminal_vertices:
                    inflow = pulp.lpSum(x[i, j] for j in delta_plus[(v, t)])
                    outflow = pulp.lpSum(x[i, j] for j in delta_minus[(v, t)])
                    prob += inflow == outflow, f"flow_conservation_{i}_{v}_{t}"
    
    # Constraint (3): Flow at source and destination
    for i in range(n):
        s_i, g_i, et_i, lt_i = S[i]
        # Source at t=0: exactly one outgoing edge
        prob += pulp.lpSum(x[i, j] for j in delta_minus[(s_i, 0)]) == 1, f"source_{i}"
        # Destination: total inflow = 1
        prob += pulp.lpSum(x[i, j] for j in delta_plus[(g_i, T - 1)]) == 1, f"destination_{i}"
    
    # Constraint (4): Vertex conflict - at most one agent per vertex at any time
    for v in range(V):
        for t in range(T):
            prob += pulp.lpSum(x[i, j] for i in range(n) for j in delta_plus[(v, t)]) <= 1, \
                    f"vertex_conflict_{v}_{t}"
    
    # Constraint (5): Edge conflict - prevent swapping positions
    for t in range(T - 1):
        for u, v in E:
            if (u, t, v, t + 1) in edge_to_idx and (v, t, u, t + 1) in edge_to_idx:
                j1 = edge_to_idx[(u, t, v, t + 1)]
                j2 = edge_to_idx[(v, t, u, t + 1)]
                prob += pulp.lpSum(x[i, j1] for i in range(n)) + \
                        pulp.lpSum(x[i, j2] for i in range(n)) <= 1, \
                        f"edge_conflict_{u}_{v}_{t}"
    
    # Link x_arrival to x
    for i in range(n):
        s_i, g_i, et_i, lt_i = S[i]
        for t in range(T):
            incoming_to_goal = delta_plus[(g_i, t)]
            if incoming_to_goal:
                prob += x_arrival[i, t] == pulp.lpSum(x[i, j] for j in incoming_to_goal), \
                        f"arrival_link_{i}_{t}"
    
    # Constraint (6): Relationship between x_arrival and y
    for i in range(n):
        for t in range(T - 1):
            # y is monotonically increasing
            prob += y[i, t] <= y[i, t + 1], f"y_monotonic_{i}_{t}"
            # y[i,t] <= x_arrival[i,t]
            prob += y[i, t] <= x_arrival[i, t], f"y_arrival_1_{i}_{t}"
            # y[i,t+1] - y[i,t] + x_arrival[i,t] <= 1
            prob += y[i, t + 1] - y[i, t] + x_arrival[i, t] <= 1, f"y_arrival_2_{i}_{t}"
    
    # Constraint (7): At final time step
    for i in range(n):
        prob += y[i, T - 1] == x_arrival[i, T - 1], f"y_final_{i}"
    
    # Objective: Maximize average customer satisfaction
    for i in range(n):
        s_i, g_i, et_i, lt_i = S[i]
        arrival_time = T - pulp.lpSum(y[i, t] for t in range(T))
        cs_raw = (lt_i - arrival_time) / (lt_i - et_i)

        # cs_raw >= 1 then cs = 1
        prob += cs_raw - 1 >= -M * (1 - b[i, 0]), f"cs_case1_a_{i}"
        prob += cs[i] - 1 >= -M * (1 - b[i, 0]), f"cs_case1_b_{i}"
        prob += cs[i] - 1 <= M * (1 - b[i, 0]), f"cs_case1_c_{i}"

        # 0 <= cs_raw <= 1 then cs = cs_raw
        prob += cs_raw - 0 >= -M * (1 - b[i, 1]), f"cs_case2_a_{i}"
        prob += cs_raw - 1 <= M * (1 - b[i, 1]), f"cs_case2_b_{i}"
        prob += cs[i] - cs_raw >= -M * (1 - b[i, 1]), f"cs_case2_c_{i}"
        prob += cs[i] - cs_raw <= M * (1 - b[i, 1]), f"cs_case2_d_{i}"

        # cs_raw <= 0 then cs = 0
        prob += cs_raw - 0 <= M * (1 - b[i, 2]), f"cs_case3_a_{i}"
        prob += cs[i] - 0 >= -M * (1 - b[i, 2]), f"cs_case3_b_{i}"
        prob += cs[i] - 0 <= M * (1 - b[i, 2]), f"cs_case3_c_{i}"

        prob += pulp.lpSum(b[i, j] for j in range(3)) == 1, f"cs_one_case_{i}"
    
    # Objective function
    prob += pulp.lpSum(cs[i] for i in range(n)), "Total_CS"
    
    # Solve
    if time_limit_in_seconds != -1:
        solver = pulp.PULP_CBC_CMD(timeLimit=time_limit_in_seconds, msg=0)
    else:
        solver = pulp.PULP_CBC_CMD(msg=0)
    
    prob.solve(solver)
    
    # Check status
    status = pulp.LpStatus[prob.status]
    
    if status == 'Optimal' or status == 'Feasible':
        print(f'Solution found! Status: {status}')
        print(f'Objective value: {pulp.value(prob.objective)}')
        
        # Extract paths
        paths = []
        for i in range(n):
            path = []
            for j in range(num_edges):
                if pulp.value(x[i, j]) > 0.5:
                    u, t_u, v, t_v = time_expanded_edges[j]
                    path.append(u)
            path += [S[i][1]]
            paths.append(path)
            
            # Calculate arrival time
            arrival_time = T - sum(pulp.value(y[i, t]) for t in range(T))
            cs_value = pulp.value(cs[i])
            
            print(f'Agent {i}: arrival time = {arrival_time:.1f}, CS = {cs_value:.3f}')

        return {
            'status': status.lower(),
            'objective': pulp.value(prob.objective),
            'paths': paths
        }
    else:
        print(f'No solution found. Status: {status}')
        return None