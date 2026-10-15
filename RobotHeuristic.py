
def manhattan_distance(p1, p2):
    return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])


def robot_to_box_heuristic(state):

    # 1. Lọc ra danh sách các thùng CHƯA ở ô đích
    unplaced_boxes = [box for box in state.boxes if box not in state.storage]

    # Nếu tất cả các thùng đã vào vị trí đích, chi phí tiếp cận thùng bằng 0
    if not unplaced_boxes:
        return 0

    total_robot_dist = 0

    # 2. Với mỗi thùng chưa vào đích, tìm khoảng cách ngắn nhất đến một robot bất kỳ
    for box in unplaced_boxes:
        # Tìm khoảng cách Manhattan nhỏ nhất từ thùng này tới tất cả các robot
        min_dist = min(manhattan_distance(robot, box) for robot in state.robots)

        # Trừ 1 vì Robot chỉ cần di chuyển tới ô KỀ BÊN thùng để bắt đầu đẩry,
        # không cần đứng trùng ô với thùng.
        # max(0, min_dist - 1) đảm bảo chi phí không bị âm khi robot đã đứng kế bên thùng.
        total_robot_dist += max(0, min_dist - 1)

    return total_robot_dist


def box_to_storage_heuristic(state):
    """Heuristic chính:

    Tính tổng khoảng cách Manhattan từ các Thùng chưa đặt đến các ô Đích (storage)
    chưa có thùng.
    """
    unplaced_boxes = [box for box in state.boxes if box not in state.storage]
    empty_storages = [
        storage for storage in state.storage if storage not in state.boxes
    ]

    if not unplaced_boxes or not empty_storages:
        return 0

    total_box_dist = 0
    for box in unplaced_boxes:
        # Tìm ô đích chưa occupied gần nhất cho thùng này
        min_storage_dist = min(
            manhattan_distance(box, storage) for storage in empty_storages
        )
        total_box_dist += min_storage_dist

    return total_box_dist


def combined_heuristic(state):
    """Hàm Heuristic tổng hợp (dùng cho A* trong solution.py):

    H(n) = H_box_to_storage(n) + H_robot_to_box(n)
    """
    return box_to_storage_heuristic(state) + robot_to_box_heuristic(state)