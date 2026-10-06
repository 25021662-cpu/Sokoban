import time
from sokoban import sokoban_goal_state

def dfs_search(initial_state, timebound=120):
    """
    Thắng code thuật toán DFS ở đây.
    Đầu vào: 
        - initial_state: object trạng thái đầu (SokobanState)
        - timebound: giới hạn thời gian (giây)
    Đầu ra: 
        - Trạng thái đích (SokobanState) nếu tìm thấy, ngược lại trả về False.
    """
    start_time = time.time()
    
    # Gợi ý: Dùng list thông thường làm Stack (ngăn xếp) cho DFS
    stack = [initial_state]
    # Gợi ý: Dùng set để lưu hashable_state của các trạng thái đã đi qua (tránh lặp)
    explored = set([initial_state.hashable_state()])
    
    while stack:
        # Kiểm tra quá thời gian
        if time.time() - start_time > timebound:
            return False
            
        # BƯỚC 1: Lấy trạng thái hiện tại ra khỏi đỉnh stack (pop)
        # current = stack.pop()
        
        # BƯỚC 2: Kiểm tra đích 
        # if sokoban_goal_state(current): 
        #     return current
            
        # BƯỚC 3: Duyệt các trạng thái con:
        # for next_state in current.successors():
        #     if next_state.hashable_state() not in explored:
        #          thêm vào stack và explored...
        
        # BỎ DÒNG BREAK DƯỚI ĐÂY KHI BẠN BẮT ĐẦU CODE THẬT
        break
        
    return False
