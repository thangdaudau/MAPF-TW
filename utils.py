import glob
import os
from typing import List, Tuple, Dict, Any
from pathlib import Path


def bfs0(g: list[list[int]], s, e) -> int:
    q = [s]
    dist = [1 << 30] * len(g)
    dist[s] = 0
    while q:
        u = q.pop()
        for v in g[u]:
            if dist[v] > dist[u] + 1:
                dist[v] = dist[u] + 1
                q += [v]
    return dist[e]

def bfs1(V: int, E: list[int], s, e) -> int:
    g = [[] for _ in range(V)]
    for u, v in E:
        g[u] += [v]
        g[v] += [u]
    return bfs0(g, s, e)

import yaml
import random
def generate_input_from_yaml(input_path: str):
    """
    Returns:
        (V: int, E: list[(int, int)], S: [(int, int, int, int)], n: int, m: int)

    Expands:
        (V: vertices, E: edges, S:[(start, goal, early, lately)], n: rows, m: columns)

    Constrains:
        early < lately

    """
    V = 0
    E = []
    S = []

    with open(input_path) as input_file:
        input_info = yaml.load(input_file, Loader=yaml.FullLoader)

    n, m = input_info['map']['dimensions']

    V = n * m

    for i in range(n):
        for j in range(m):
            if (j, i) not in input_info['map']['obstacles']:
                for di in range(2):
                    dj = 1 - di
                    ii = i + di
                    jj = j + dj
                    if 0 <= ii < n and 0 <= jj < m and (jj, ii) not in input_info['map']['obstacles']:
                        E += [(i * m + j, ii * m + jj)]

    for agent in input_info['agents']:
        x, y = agent['start']
        xx, yy = agent['goal']
        u = y * m + x
        v = yy * m + xx
        dist = bfs1(V, E, u, v)
        et = int(dist * random.uniform(0, 2))
        lt = int(dist * random.uniform(2, 3) + 1) + 1
        S += [(u, v, et, lt)]
    
    # time_window_path = input_path.replace('input', 'time_window')

    # print(f'Generated time_window for {input_path} in {time_window_path}')
    print(f'Time window for {input_path}:')
    for i, (_, _, et, lt) in enumerate(S):
        print(f'agent{i}: [{et}, {lt}]')

    return V, E, S, n, m



def save_output(output_path: str, agent_paths: list[list[(int, int)]]):
    """
    agent_paths: list của các list vertex id

    m_cols: số cột trong map (để convert vertex -> (x, y))

    """
    d = {'schedule': {}}

    for i, path in enumerate(agent_paths):
        schedule = []
        for t, v in enumerate(path):
            x, y = v
            schedule.append({'t': t, 'x': x, 'y': y})
        d['schedule'][f'agent{i}'] = schedule

    with open(output_path, "w") as f:
        yaml.dump(d, f, sort_keys=False)


def generate_input_for_stdin(V: int, E: list[tuple[int, int]], S: list[tuple[int, int, int, int]], n, m):
    ret = [str(-1)]
    ret += [f'{n} {m}']
    ret += [str(V)]
    ret += [str(len(E))]
    for u, v in E:
        ret += [f'{u} {v}']
    ret += [str(len(S))]
    for s, g, et, lt in S:
        ret += [f'{s} {g} {et} {lt}']
    return '\n'.join(ret)


def convert_map_to_yaml(relative_map_path: str, yaml_output_filename: str):
    """
    Đọc nội dung file map từ đường dẫn tương đối và chuyển đổi thành định dạng YAML
    với danh sách các chướng ngại vật (obstacles) sử dụng tuple Python thuần túy (x, y).

    Args:
        relative_map_path (str): Đường dẫn tương đối đến file .map đầu vào.
        yaml_output_filename (str): Tên file YAML đầu ra để lưu kết quả.
    """
    print(f"Bắt đầu xử lý file map tại đường dẫn: **{relative_map_path}**")

    # 1. Đọc nội dung file
    try:
        map_path = Path(relative_map_path)
        map_file_content = map_path.read_text(encoding='utf-8')
    except FileNotFoundError:
        print(f"LỖI: Không tìm thấy file tại đường dẫn: {relative_map_path}. Kiểm tra lại path!")
        return
    except Exception as e:
        print(f"LỖI khi đọc file: {e}")
        return

    # --- PHẦN XỬ LÝ MAP ---
    
    lines = map_file_content.strip().split('\n')
    
    map_data: Dict[str, Any] = {
        'agents': [],
        'map': {
            'dimensions': [],
            'obstacles': []
        },
        'dynamic_obstacles': {}
    }

    width = 0
    height = 0
    map_lines: List[str] = []

    for line in lines:
        if line.startswith('width'):
            width = int(line.split()[1])
        elif line.startswith('height'):
            height = int(line.split()[1])
        elif line.startswith('map'):
            map_index = lines.index(line) + 1
            map_lines = lines[map_index:]
            break 

    if not map_lines:
        print("LỖI: Không tìm thấy khối 'map' hoặc kích thước map.")
        return

    # Cập nhật kích thước thực tế
    height = len(map_lines)
    if height > 0:
        width = len(map_lines[0])
        map_data['map']['dimensions'] = [height, width]

    print(f"Kích thước Map được phát hiện: **{height}x{width}**")
    
    # 2. Quét map để tìm chướng ngại vật (@) và đảo ngược tọa độ Y
    obstacles: List[Tuple[int, int]] = []
    
    for y_map_index, row in enumerate(map_lines):
        for x_coord, char in enumerate(row):
            if char != '.':
                # Đảo ngược trục Y: y_coord = height - 1 - y_map_index
                y_coord = height - 1 - y_map_index
                obstacles.append((x_coord, y_coord))

    # Gán thẳng danh sách tuple vào map_data
    map_data['map']['obstacles'] = obstacles
    
    print(f"Tìm thấy tổng cộng **{len(obstacles)}** chướng ngại vật.")
    
    # 3. Ghi ra file YAML (Sử dụng dumper mặc định)
    try:
        with open(yaml_output_filename, 'w') as f:
            yaml.dump(map_data, f, sort_keys=False, default_flow_style=None)
                      
        print(f"✅ Đã chuyển đổi thành công! Kết quả được lưu tại: **{yaml_output_filename}**")
    except Exception as e:
        print(f"LỖI khi ghi file YAML: {e}")