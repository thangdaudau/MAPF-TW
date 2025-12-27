from typing import List, Tuple, Any, Optional
from enum import Enum

# --- Enums (Ánh xạ từ C++) ---
class GCBSMode(Enum):
    GCBS_H = 0
    GCBS_L = 1
    GCBS_LH = 2

class ConflictHeuristic(Enum):
    NUM_CONFLICTS = 0
    NUM_CONFLICTING_AGENTS = 1
    NUM_CONFLICTING_PAIRS = 2
    VERTEX_COVER = 3
    ALTERNATING = 4

# --- Định nghĩa kiểu dữ liệu ---
# (start, goal, early, late)
AgentTuple = Tuple[int, int, int, int]
# (cost, paths)
SolutionResult = Tuple[float, List[List[int]]]

class CBS:
    def __init__(self, 
                 row: int, 
                 col: int, 
                 edges: List[Tuple[int, int]], 
                 agents: List[AgentTuple]) -> None:
        """
        Khởi tạo thuật toán CBS.
        :param row: Số hàng
        :param col: Số cột
        :param edges: Danh sách cạnh [(u, v), ...]
        :param agents: Danh sách agent [(start, goal, early, late), ...]
        """
        ...

    def set_time_limit(self, limit: int) -> None:
        """Thiết lập giới hạn thời gian chạy (giây)"""
        ...

    def solve(self, release_gil: bool = False) -> SolutionResult:
        """
        Chạy thuật toán.
        :return: Tuple (Tổng chi phí, List các đường đi).
        """
        ...

class ICBS:
    def __init__(self, 
                 row: int, 
                 col: int, 
                 edges: List[Tuple[int, int]], 
                 agents: List[AgentTuple],
                 mergeThreshold: int = 25,
                 mergeRestartActive: bool = True, 
                 maxMetaAgentSize: int = (1 << 32) - 1) -> None:
        ...

    def set_time_limit(self, limit: int) -> None: ...
    def solve(self, release_gil: bool = False) -> SolutionResult: ...

class GCBS:
    def __init__(self, 
                 row: int, 
                 col: int, 
                 edges: List[Tuple[int, int]], 
                 agents: List[AgentTuple],
                 mode: GCBSMode = GCBSMode.GCBS_LH,
                 heuristicType: ConflictHeuristic = ConflictHeuristic.NUM_CONFLICTING_PAIRS) -> None:
        ...

    def set_time_limit(self, limit: int) -> None: ...
    def solve(self, release_gil: bool = False) -> SolutionResult: ...