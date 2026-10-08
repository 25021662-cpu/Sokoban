"""Student entry point. See README.md for the API and scoring rules."""
import time
from sokoban import sokoban_goal_state

# -- IMPORT CÁC MODULE CỦA THÀNH VIÊN TRONG QUÁ TRÌNH DEV --
# KHI NỘP BÀI: Xóa phần import này và dồn toàn bộ code của các bạn vào đây.

import BFS
import DFS
import AStar

def check_early_stop(state):
    """
    Hàm kiểm tra các trường hợp bất thường để dừng sớm trước khi chạy thuật toán.
    Trả về:
    - state: Nếu bản đồ đã là đích sẵn.
    - False: Nếu bản đồ vô nghiệm.
    - None: Nếu bản đồ bình thường, cần tiếp tục chạy thuật toán tìm kiếm.
    """
    # Nếu đã là đích ngay từ đầu
    if sokoban_goal_state(state):
        return state
        
    # Số lượng thùng nhiều hơn số đích -> Vô nghiệm
    if len(state.boxes) > len(state.storage):
        return False
        
    # Không có robot nhưng thùng chưa vào đích -> Vô nghiệm
    if not state.robots:
        return False
        
    # Trạng thái bình thường, tiếp tục
    return None

def solve(initial_state, timebound=120):
    """Return a goal SokobanState with a valid parent chain, or False."""
    
    # 1. KÍCH HOẠT KIỂM TRA DỪNG SỚM
    early_stop = check_early_stop(initial_state)
    if early_stop is not None:
        return early_stop
        
    # 2. GỌI THUẬT TOÁN ĐỂ CHẠY (UNCOMMENT DÒNG TƯƠNG ỨNG ĐỂ TEST)
    
    # Chạy BFS của Nhi:
    return BFS.bfs_search(initial_state, timebound)

    # Chạy DFS của Thắng:
    return DFS.dfs_search(initial_state, timebound)
    
    # Chạy A* của Cường:
    # return AStar.astar_search(initial_state, timebound)
    
    # Tạm thời trả về False để không bị lỗi lúc chạy autograder
    return False