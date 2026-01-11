import sys
import os
import time
from typing import List, Tuple


# --- PHẦN 1: LIÊN KẾT THƯ VIỆN C++ ---
# Tìm đường dẫn: wrapper.py -> core -> MAPF-TW (root)
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# File .pyd/.so thường nằm ở: build/core/bindings
LIB_PATH = os.path.join(PROJECT_ROOT, 'build', 'core', 'bindings')

# Thêm vào path để Python tìm thấy
if os.path.exists(LIB_PATH):
    sys.path.append(LIB_PATH)
else:
    print(f"[CẢNH BÁO] Không tìm thấy thư mục build tại: {LIB_PATH}")

# Import module C++ (tên là mapf_solver như trong CMake đã đặt)
try:
    import mapf_solver
except ImportError as e:
    print(f"[LỖI] Không thể load thư viện C++ 'mapf_solver'.")
    print("1. Hãy chắc chắn bạn đã build thành công (có file .pyd hoặc .so).")
    print("2. Kiểm tra xem Python build và Python chạy có trùng version không.")
    mapf_solver = None

# Lấy Enums từ C++ (nếu load được)
if mapf_solver:
    GCBSMode = mapf_solver.GCBSMode
    ConflictHeuristic = mapf_solver.ConflictHeuristic
else:
    # Fallback class để IDE không báo lỗi đỏ lòm khi chưa build
    class GCBSMode:
        GCBS_H, GCBS_L, GCBS_LH = 0, 1, 2
    class ConflictHeuristic:
        NUM_CONFLICTS, NUM_CONFLICTING_AGENTS, NUM_CONFLICTING_PAIRS, VERTEX_COVER, ALTERNATING = 0, 1, 2, 3, 4


# --- PHẦN 2: WRAPPER CODE ---

time_limit_in_seconds = -1
release_gil = False

def set_time_limit(seconds: int):
    global time_limit_in_seconds
    time_limit_in_seconds = seconds

def set_release_gil(is_release_gil: bool):
    global release_gil
    release_gil = is_release_gil

if __name__ != '__main__':
    from core.ILPBased import MIP_MAPF_TW
else:
    from ILPBased import MIP_MAPF_TW
def ILP(V: int, E: list[Tuple[int, int]], S: list[Tuple[int, int, int, int]], T: int):
    # Hàm cũ của ông, giữ nguyên
    res = MIP_MAPF_TW(V, E, S, T, time_limit_in_seconds)
    return res['objective'], res['paths'] if res else None

# --- CÁC HÀM MỚI ---

def _ensure_cpp_lib():
    if mapf_solver is None:
        raise RuntimeError("Thư viện C++ mapf_solver chưa được load!")

def _run_solver(solver):
    """Helper function để chạy solver và trả về đúng format"""
    if time_limit_in_seconds > 0:
        solver.set_time_limit(time_limit_in_seconds)
    
    # C++ binding giờ trả về std::pair<double, vector<vector<int>>>
    # Python sẽ nhận được tuple (cost, paths)
    cost, paths = solver.solve(release_gil)
    return None if not paths else (cost, paths)

def CBS(n: int, m: int, E: list[Tuple[int, int]], S: list[Tuple[int, int, int, int]]):
    """
    Gọi thuật toán CBS từ C++
    Trả về: (cost, paths)
    """
    _ensure_cpp_lib()
    
    # Khởi tạo Solver từ C++
    solver = mapf_solver.CBS(n, m, E, S)
    
    return _run_solver(solver)

def ICBS(n: int, m: int, E: list[Tuple[int, int]], S: list[Tuple[int, int, int, int]],
         mergeThreshold: int = 25,
         mergeRestartActive: bool = True, 
         maxMetaAgentSize: int = (1 << 32) - 1):
    """
    Gọi thuật toán ICBS từ C++
    Trả về: (cost, paths)
    """
    _ensure_cpp_lib()
    
    # Khởi tạo Solver ICBS với tham số tùy chọn
    solver = mapf_solver.ICBS(n, m, E, S, mergeThreshold, mergeRestartActive, maxMetaAgentSize)

    return _run_solver(solver)

def GCBS(n: int, m: int, E: list[Tuple[int, int]], S: list[Tuple[int, int, int, int]],
         mode: GCBSMode = GCBSMode.GCBS_LH,
         heuristicType: ConflictHeuristic = ConflictHeuristic.NUM_CONFLICTING_PAIRS):
    """
    Gọi thuật toán GCBS từ C++
    Trả về: (cost, paths)
    """
    _ensure_cpp_lib()

    # Khởi tạo Solver GCBS với tham số tùy chọn
    solver = mapf_solver.GCBS(n, m, E, S, mode, heuristicType)

    return _run_solver(solver)