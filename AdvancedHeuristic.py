"""
AdvancedHeuristic.py - Heuristic ghép cặp 1-1 tối ưu giữa Thùng và Đích (Box -> Storage).

Nâng cấp kết hợp True BFS Distance Matrix từ mapper.py:
- Thay vì dùng Manhattan (|x1-x2| + |y1-y2|) vốn bỏ qua vật cản,
- Sử dụng khoảng cách thực tế quanh tường được pre-compute trong mapper.py.
- Tự động phát hiện bế tắc nếu một thùng nằm ở vùng không thể tiếp cận bất kỳ ô đích nào.
"""

import mapper

_hbox_cache = {}


def manhattan_distance(box, goal):
    """Fallback tính khoảng cách Manhattan nếu không có distance_matrix."""
    return abs(box[0] - goal[0]) + abs(box[1] - goal[1])


def advanced_heuristic(state, dist_matrix=None):
    """
    Tính heuristic ghép cặp 1-1 giữa Thùng và Đích dựa trên khoảng cách đi bộ thực tế.
    """
    cache_key = (state.boxes, state.storage, state.obstacles)
    h_box = _hbox_cache.get(cache_key)
    if h_box is None:
        h_box = _compute_advanced_heuristic(state, dist_matrix)
        _hbox_cache[cache_key] = h_box
    return h_box


def _compute_advanced_heuristic(state, dist_matrix=None):
    boxes = sorted(state.boxes)
    goals = sorted(state.storage)

    if not boxes:
        return 0

    if len(goals) < len(boxes):
        return float("inf")

    # Lấy ma trận khoảng cách thực tế từ mapper (đã có cache nội bộ trong mapper)
    if dist_matrix is None:
        try:
            dist_matrix = mapper.compute_distance_matrix(state)
        except Exception:
            dist_matrix = None

    pairs = []

    # Tính khoảng cách thực tế (quanh tường) từ mỗi thùng tới mỗi đích
    for i, box in enumerate(boxes):
        for j, goal in enumerate(goals):
            if dist_matrix is not None and goal in dist_matrix:
                d = dist_matrix[goal].get(box, float("inf"))
            else:
                d = manhattan_distance(box, goal)
            pairs.append((d, i, j))

    # Sắp xếp theo khoảng cách tăng dần
    pairs.sort()

    used_boxes = set()
    used_goals = set()
    total = 0

    # Greedy matching: chọn cặp có khoảng cách nhỏ nhất chưa được gán
    for distance, i, j in pairs:
        if distance == float("inf"):
            continue
        if i in used_boxes or j in used_goals:
            continue

        used_boxes.add(i)
        used_goals.add(j)
        total += distance

        if len(used_boxes) == len(boxes):
            break

    # Nếu có thùng không thể ghép được với bất kỳ đích nào khả thi -> Bế tắc (Deadlock)
    if len(used_boxes) < len(boxes):
        return float("inf")

    return total
