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


def min_cost_bipartite_matching(cost_matrix):
    """
    Thuật toán Kuhn-Munkres (Hungarian) chuẩn O(V^3) thuần Python, không import ngoài.
    Tìm gán cặp 1-1 tối ưu toàn cục giữa Thùng và Đích (đảm bảo Admissible & Gradient chuẩn xác).
    """
    n = len(cost_matrix)
    if n == 0:
        return 0, ()
    m = len(cost_matrix[0])
    if n > m:
        transposed = [[cost_matrix[i][j] for i in range(n)] for j in range(m)]
        c, pairs = min_cost_bipartite_matching(transposed)
        return c, tuple((j, i) for i, j in pairs)

    u = [0] * (n + 1)
    v = [0] * (m + 1)
    p = [0] * (m + 1)
    way = [0] * (m + 1)

    for i in range(1, n + 1):
        p[0] = i
        j0 = 0
        minv = [float('inf')] * (m + 1)
        used = [False] * (m + 1)
        while True:
            used[j0] = True
            i0 = p[j0]
            delta = float('inf')
            j1 = 0
            for j in range(1, m + 1):
                if not used[j]:
                    cur = cost_matrix[i0 - 1][j - 1] - u[i0] - v[j]
                    if cur < minv[j]:
                        minv[j] = cur
                        way[j] = j0
                    if minv[j] < delta:
                        delta = minv[j]
                        j1 = j
            for j in range(m + 1):
                if used[j]:
                    u[p[j]] += delta
                    v[j] -= delta
                else:
                    minv[j] -= delta
            j0 = j1
            if p[j0] == 0:
                break
        while True:
            j1 = way[j0]
            p[j0] = p[j1]
            j0 = j1
            if j0 == 0:
                break

    matching = [0] * n
    for j in range(1, m + 1):
        if p[j] != 0:
            matching[p[j] - 1] = j - 1

    total_cost = sum(cost_matrix[i][matching[i]] for i in range(n))
    return total_cost, tuple((i, matching[i]) for i in range(n))


_UNREACHABLE_COST = 10**8

def _compute_advanced_heuristic(state, dist_matrix=None):
    boxes = sorted(state.boxes)
    goals = sorted(state.storage)
    nb = len(boxes)
    ng = len(goals)

    if nb == 0:
        return 0

    if ng < nb:
        return float("inf")

    if dist_matrix is None:
        try:
            dist_matrix = mapper.compute_distance_matrix(state)
        except Exception:
            dist_matrix = None

    cost_matrix = []
    for box in boxes:
        row = []
        for goal in goals:
            if dist_matrix is not None and goal in dist_matrix:
                d = dist_matrix[goal].get(box, _UNREACHABLE_COST)
            else:
                d = manhattan_distance(box, goal)
            row.append(d)
        cost_matrix.append(row)

    for r in range(nb):
        if all(cost_matrix[r][c] >= _UNREACHABLE_COST for c in range(ng)):
            return float("inf")

    cost, pairs = min_cost_bipartite_matching(cost_matrix)
    if any(cost_matrix[r][c] >= _UNREACHABLE_COST for r, c in pairs):
        return float("inf")

    return cost
