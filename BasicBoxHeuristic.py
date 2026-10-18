# tính khoảng cách Manhattan
def manhattan_distance(box, storage):
    return abs(box[0] - storage[0]) + abs(box[1] - storage[1])

# tính tổng khoảng cách từ mỗi thùng đến ô điểm lưu trữ gần nhất
def basic_box_heuristic(state):
    if not state.boxes:
        return 0
    if not state.storage:
        return float("inf")

    return sum(
        min(manhattan_distance(box, storage) for storage in state.storage)
        for box in state.boxes
    )


