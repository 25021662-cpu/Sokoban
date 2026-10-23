import heapq
import time

import mapper
import RobotHeuristic as RH
from AdvancedHeuristic import advanced_heuristic
from sokoban import sokoban_goal_state


# Class bọc (wrapper) để so sánh các Node trong PriorityQueue dựa vào giá trị f_value
class Node:
    def __init__(self, state, f_value, h_value):
        self.state = state
        self.f_value = f_value
        self.h_value = h_value
    
    def __lt__(self, other):
        # TIE-BREAKING CỰC KỲ QUAN TRỌNG:
        # Nếu f(n) bằng nhau (đặc biệt phổ biến khi nhiều agent không cản đường nhau),
        # ưu tiên mở trạng thái có h(n) NHỎ HƠN (tức là trạng thái có vẻ gần đích hơn).
        # Điều này ép A* đi sâu (Depth-first) thay vì loang rộng như BFS, chống nổ RAM.
        if self.f_value == other.f_value:
            return self.h_value < other.h_value
        return self.f_value < other.f_value


def heuristic(state):
    """
    Hàm Heuristic tổng hợp kết hợp 2 file:
    1. AdvancedHeuristic (Thắng): Ghép cặp 1-1 giữa Thùng và Ô đích (Box -> Storage).
    2. RobotHeuristic (Cường): Chi phí điều phối Robot tiếp cận Thùng chưa vào đích (Robot -> Box).
    """
    
    # 1. Chi phí đẩy Thùng vào Đích (Ghép 1-1 tối ưu)
    h_box = advanced_heuristic(state)
    if h_box == float("inf"):
        return float("inf")

    # 2. Chi phí Robot tiếp cận Thùng gần nhất
    h_robot = RH.robot_to_box_heuristic(state, mode="guided")

    return h_box + h_robot
    

def astar_search(initial_state, w=1.2, timebound=120, heuristic_fn=heuristic):
    """
    Thuật toán Weighted A* (WA*):
        f(n) = g(n) + w * h(n)
    Đầu vào: 
        - initial_state: object trạng thái đầu (SokobanState)
        - w: hệ số trọng số cho heuristic (mặc định w=1.2, w=1.0 là A* chuẩn)
        - timebound: giới hạn thời gian (giây)
        - heuristic_fn: hàm heuristic sử dụng (mặc định là heuristic tổng hợp)
    Đầu ra: 
        - Trạng thái đích (SokobanState) nếu tìm thấy, ngược lại trả về False.
    """
    
    map_info = mapper.get_map_info(initial_state)
    dead_squares = map_info.dead_squares
    
    # Hàng đợi ưu tiên (Priority Queue) của A*
    pq = []
    
    # Trạng thái khởi tạo
    h_initial = heuristic_fn(initial_state)
    if h_initial == float("inf"):
        return False

    f_initial = initial_state.gval + w * h_initial
    heapq.heappush(pq, Node(initial_state, f_initial, h_initial))

    # Lưu chi phí gval nhỏ nhất từng đến state đó
    explored = {initial_state.hashable_state(): initial_state.gval}

    while pq:
        # BƯỚC 1: Lấy Node có f_value nhỏ nhất ra
        current_node = heapq.heappop(pq)
        current_state = current_node.state

        if current_state.gval > explored.get(current_state.hashable_state(), float("inf")):
            continue
        # bỏ qua nếu thấy gval nhỏ hơn

        # BƯỚC 2: Kiểm tra đích: 
        if sokoban_goal_state(current_state):
            return current_state
            
        # BƯỚC 3: Duyệt các trạng thái con và tính toán hàm f:
        for next_state in current_state.successors():
            next_key = next_state.hashable_state()
            new_g = next_state.gval

            if next_key not in explored or new_g < explored[next_key]:
                if mapper.is_deadlock(next_state, map_info):
                    continue
                
                h_val = heuristic_fn(next_state)
                if h_val == float("inf"):
                    continue

                explored[next_key] = new_g
                f_val = new_g + w * h_val
                heapq.heappush(pq, Node(next_state, f_val, h_val))

    return False
