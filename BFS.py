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
    start_time = time.time() #Ghi lại thời điểm bắt đầu
    
    # Gợi ý: Dùng deque để làm hàng đợi (queue) cho BFS
    queue = deque([initial_state])
    # Gợi ý: Dùng set để lưu hashable_state của các trạng thái đã đi qua (tránh lặp)
    explored = set([initial_state.hashable_state()])  #Tạo hàng đợi và đưa trạng thái bắt đầu vào
    
    while queue:
        # Kiểm tra quá thời gian
        if time.time() - start_time > timebound:
            return False

        current = queue.popleft()  #Lấy trạng thái đầu queue(popleft để đi theo thứ tự FIFO của BFS)

        if sokoban_goal_state(current):
            return current    #Nếu là đích thì dừng

        for next_state in current.successors():  #loops
            key = next_state.hashable_state() 
            if key not in explored: 
                explored.add(key)
                queue.append(next_state)  #đệ quy

        
        
    return False
