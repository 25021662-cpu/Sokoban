import time
from collections import deque
from sokoban import sokoban_goal_state

def bfs_search(initial_state, timebound=120):
    """
    Nhi code thuật toán BFS ở đây.
    Đầu vào: 
        - initial_state: object trạng thái đầu (SokobanState)
        - timebound: giới hạn thời gian (giây)
    Đầu ra: 
        - Trạng thái đích (SokobanState) nếu tìm thấy, ngược lại trả về False.
    """
    start_time = time.time()
    
    # Gợi ý: Dùng deque để làm hàng đợi (queue) cho BFS
    queue = deque([initial_state])
    # Gợi ý: Dùng set để lưu hashable_state của các trạng thái đã đi qua (tránh lặp)
    explored = set([initial_state.hashable_state()])
    
    while queue:
        # Kiểm tra quá thời gian
        if time.time() - start_time > timebound:
            return False
            
        # BƯỚC 1: Lấy trạng thái hiện tại ra khỏi queue (popleft)
        # current = queue.popleft()
        
        # BƯỚC 2: Kiểm tra xem đã đến đích chưa
        # if sokoban_goal_state(current): 
        #     return current
            
        # BƯỚC 3: Duyệt các trạng thái con (các nước đi kế tiếp)
        # for next_state in current.successors():
        #     if next_state.hashable_state() not in explored:
        #          thêm vào queue và explored...
        
        # BỎ DÒNG BREAK DƯỚI ĐÂY KHI BẠN BẮT ĐẦU CODE THẬT
        break
        
    return False
