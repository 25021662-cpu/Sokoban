import time
import heapq
from sokoban import sokoban_goal_state

# Class bọc (wrapper) để so sánh các Node trong PriorityQueue dựa vào giá trị f_value
class Node:
    def __init__(self, state, f_value):
        self.state = state
        self.f_value = f_value
    
    # Hàm ghi đè toán tử bé hơn (<) để heapq biết cách sắp xếp
    def __lt__(self, other):
        return self.f_value < other.f_value

def dummy_heuristic(state):
    """
    Tạm thời trả về 0 để A* chạy giống hệt thuật toán UCS (Uniform Cost Search)
    Giai đoạn 2 nhóm sẽ viết logic tính toán khoảng cách Manhattan vào đây.
    """
    return 0

def astar_search(initial_state, timebound=120):
    """
    Cường xây dựng bộ khung thuật toán A* (A-Star) ở đây.
    Đầu vào: 
        - initial_state: object trạng thái đầu (SokobanState)
        - timebound: giới hạn thời gian (giây)
    Đầu ra: 
        - Trạng thái đích (SokobanState) nếu tìm thấy, ngược lại trả về False.
    """
    start_time = time.time()
    
    # Hàng đợi ưu tiên (Priority Queue) của A*
    pq = []
    
    # Trạng thái khởi tạo
    f_initial = initial_state.gval + dummy_heuristic(initial_state)
    heapq.heappush(pq, Node(initial_state, f_initial))
    
    # Lưu các trạng thái đã đi qua (với A* thường dùng dict để lưu kèm cost nhỏ nhất nếu muốn tối ưu thêm)
    explored = {initial_state.hashable_state(): initial_state.gval}

    while pq:
        # Kiểm tra quá thời gian
        if time.time() - start_time > timebound:
            return False
            
        # BƯỚC 1: Lấy Node có f_value nhỏ nhất ra
        current_node = heapq.heappop(pq)
        current_state = current_node.state

        if current_state.gval > explored.get(current_state.hashable_state(), float("inf")): continue
        # bỏ qua nếu thấy gval nhỏ hơn

        # BƯỚC 2: Kiểm tra đích: 
        if sokoban_goal_state(current_state):
             return current_state
            
        # BƯỚC 3: Duyệt các trạng thái con và tính toán hàm f:
        for next_state in current_state.successors():
            next_key = next_state.hashable_state()
            new_g = next_state.gval
            if next_key not in explored or new_g < explored[next_key]:
              explored[next_key] = new_g
              f_val = new_g + dummy_heuristic(next_state)
              heapq.heappush(pq, Node(next_state, f_val))

    return False
