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
    # thời gian bắt đầu
    start_time = time.time()

    # lập Stack chứa các trạng thái đang chờ được duyệt.
    stack = [initial_state]

    # explored lưu các trạng thái đã phát hiện.
    # Chỉ lưu hashable_state để so sánh nhanh hơn.
    explored = {initial_state.hashable_state()}

    # =========================
    # 2. VÒNG LẶP DFS
    # =========================

    while stack:

        # Kiểm tra giới hạn thời gian
        if time.time() - start_time >= timebound:
            return False

        # =========================
        # 3. LẤY STATE TRÊN ĐỈNH STACK
        # =========================

        current_state = stack.pop()

        # =========================
        # 4. KIỂM TRA GOAL
        # =========================

        if sokoban_goal_state(current_state):
            return current_state

        # =========================
        # 5. SINH CÁC STATE CON
        # =========================

        successors = current_state.successors()

        # =========================
        # 6. ĐƯA STATE CHƯA XÉT VÀO STACK
        # =========================

        for next_state in successors:

            state_key = next_state.hashable_state()

            # Chỉ xét trạng thái chưa từng gặp
            if state_key not in explored:
                # Đánh dấu ngay khi phát hiện
                # để tránh cùng một state bị thêm nhiều lần.
                explored.add(state_key)

                # Thêm vào cuối stack.
                stack.append(next_state)

    # =========================
    # 7. KHÔNG TÌM THẤY LỜI GIẢI
    # =========================

    return False