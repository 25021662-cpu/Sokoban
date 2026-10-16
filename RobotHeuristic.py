
ROBOT_MODE = "guided"        # chế độ mặc định của combined_heuristic
_MAX_BOXES_FOR_DP = 8        # quá ngưỡng này thì ghép tham lam thay vì quy hoạch động
_MAX_CACHE = 200000
_assign_cache = {}


def manhattan_distance(p1, p2):
    return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])


def approach_cost(robot, box):
    """Số bước đi bộ tối thiểu để robot đứng được CẠNH thùng (đã trừ 1 bước đẩy)."""
    return max(0, manhattan_distance(robot, box) - 1)


def _best_assignment(robots, boxes):
    """
    robots: tuple vị trí robot; boxes: tuple vị trí thùng chưa vào đích.
    Mỗi robot nhận tối đa 1 thùng, mỗi thùng có tối đa 1 robot, đủ min(R, B) cặp
    với tổng chi phí tiếp cận nhỏ nhất. Trả về (tổng chi phí, tuple các cặp (robot, thùng)).
    """
    n_r, n_b = len(robots), len(boxes)
    k = min(n_r, n_b)

    if n_b <= _MAX_BOXES_FOR_DP:
        layer = {0: (0, ())}                      # mask thùng đã có robot -> (chi phí, các cặp)
        for r in robots:
            nxt = {}
            for mask, (c, pairs) in layer.items():
                if mask not in nxt or c < nxt[mask][0]:
                    nxt[mask] = (c, pairs)        # robot r rảnh rỗi
                for j in range(n_b):
                    if not (mask >> j) & 1:
                        nm = mask | (1 << j)
                        nc = c + approach_cost(r, boxes[j])
                        if nm not in nxt or nc < nxt[nm][0]:
                            nxt[nm] = (nc, pairs + ((r, boxes[j]),))
            layer = nxt
        return min((v for m, v in layer.items() if bin(m).count("1") == k),
                   key=lambda v: v[0])

    # Quá nhiều thùng: ghép tham lam theo chi phí tăng dần
    candidates = sorted((approach_cost(r, b), r, b) for r in robots for b in boxes)
    used_r, used_b, total, pairs = set(), set(), 0, []
    for c, r, b in candidates:
        if r in used_r or b in used_b:
            continue
        used_r.add(r)
        used_b.add(b)
        total += c
        pairs.append((r, b))
        if len(pairs) == k:
            break
    return total, tuple(pairs)


def robot_box_assignment(state):
    """
    Trả về (tổng chi phí tiếp cận, các cặp (robot, thùng) được giao việc, các robot rảnh rỗi).
    Dùng để debug/in ra xem robot nào được giao thùng nào.
    """
    robots = tuple(sorted(state.robots))
    unplaced = tuple(sorted(b for b in state.boxes if b not in state.storage))
    if not unplaced or not robots:
        return 0, (), robots                      # không còn việc: mọi robot đều rảnh rỗi

    key = (robots, unplaced)
    cached = _assign_cache.get(key)
    if cached is None:
        if len(_assign_cache) > _MAX_CACHE:
            _assign_cache.clear()
        cached = _best_assignment(robots, unplaced)
        _assign_cache[key] = cached
    cost, pairs = cached
    busy = {r for r, _ in pairs}
    idle = tuple(r for r in robots if r not in busy)
    return cost, pairs, idle


def robot_to_box_guided(state):
    """Chế độ điều phối: robot rảnh rỗi đi tới thùng chưa vào đích (ghép tối ưu)."""
    return robot_box_assignment(state)[0]


def robot_to_box_admissible(state):
    """Chế độ an toàn: min theo (robot, thùng), luôn là cận dưới của số bước đi bộ còn lại."""
    unplaced = [b for b in state.boxes if b not in state.storage]
    if not unplaced or not state.robots:
        return 0          # đã là đích, hoặc không có robot (tránh min() của dãy rỗng)
    return min(
        approach_cost(r, b)
        for r in state.robots
        for b in state.boxes      # TẤT CẢ thùng, không chỉ thùng chưa vào đích
    )


def robot_to_box_heuristic(state, mode=None):
    mode = mode or ROBOT_MODE
    if mode == "guided":
        return robot_to_box_guided(state)
    if mode == "admissible":
        return robot_to_box_admissible(state)
    raise ValueError("mode phải là 'guided' hoặc 'admissible'")


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


def combined_heuristic(state, mode=None):
    """Hàm Heuristic tổng hợp :
    H(n) = H_box_to_storage(n) + H_robot_to_box(n)
    """
    return box_to_storage_heuristic(state) + robot_to_box_heuristic(state, mode)


def make_heuristic(mode):
    """Tạo hàm heuristic một tham số (state) cố định chế độ, để truyền vào A*."""
    def heuristic(state):
        return combined_heuristic(state, mode)
    return heuristic