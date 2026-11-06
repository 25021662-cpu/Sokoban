"""
solution_submit.py - Tinh hoa gộp hoàn chỉnh của bộ giải Sokoban Solver.

Toàn bộ các thuật toán được gộp vào duy nhất 1 file độc lập, không import ngoài:
1. TIỀN XỬ LÝ ĐỊA HÌNH & BẾ TẮC (MapInfo):
   - Reverse BFS (Pull Search) tìm Dead Squares tĩnh (chính xác 0% false-positive).
   - Distance Matrix: BFS tính khoảng cách đi bộ thực tế quanh tường từ các ô đích.
   - Goal Corridors: Kiểm tra bế tắc thứ tự Inside-Out trong hành lang đích.
   - Freeze Deadlock 2x2: Bắt khối kẹt cứng tường/thùng.

2. HEURISTIC DẪN ĐƯỜNG MẠNH MẼ:
   - Ghép cặp tối ưu 1-1 giữa Thùng và Đích (dùng True BFS Distance Matrix).
   - Quy hoạch động Bitmask điều phối Multi-Robot (Robot -> Box).

3. BỘ MÁY TÌM KIẾM ĐA TẦNG (CASCADING SOLVER):
   - Tầng 1 (Chính): Macro Push Weighted A* (w=2.0) với tie-breaking và Deadlock Pruning.
   - Tầng 2 (Dự phòng): Micro A*W nếu Macro Push không tìm thấy lời giải.
   - Reconstruction: Tự động bung đường đi macro thành từng bước vi mô (gval + 1) chuẩn 100% luật chơi.
"""

import heapq
import time
from collections import deque
from typing import Dict, List, Tuple, Set, Optional

from sokoban import SokobanState, sokoban_goal_state

# ═════════════════════════════════════════════════════════════════════
#  1. HẰNG SỐ & HÀM TIỆN ÍCH HÌNH HỌC / ĐỒ THỊ
# ═════════════════════════════════════════════════════════════════════

_DIRS = ((0, -1), (0, 1), (-1, 0), (1, 0))
_DIR_NAME = {(0, -1): "up", (0, 1): "down", (-1, 0): "left", (1, 0): "right"}
_MAX_BOXES_FOR_DP = 8

def manhattan_distance(p1, p2) -> int:
    return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])

def approach_cost(robot, box) -> int:
    """Số bước đi bộ tối thiểu để robot đứng cạnh thùng (trừ 1 bước đẩy)."""
    return max(0, abs(robot[0] - box[0]) + abs(robot[1] - box[1]) - 1)

def _flood(start, blocked, w, h) -> Dict[Tuple[int, int], int]:
    """BFS Flood Fill tính khoảng cách đi bộ từ start tới tất cả các ô reachable."""
    dist = {start: 0}
    q = deque([start])
    while q:
        x, y = q.popleft()
        d = dist[(x, y)]
        for dx, dy in _DIRS:
            nx, ny = x + dx, y + dy
            np_ = (nx, ny)
            if np_ in dist or np_ in blocked or not (0 <= nx < w and 0 <= ny < h):
                continue
            dist[np_] = d + 1
            q.append(np_)
    return dist

def _bfs_path(start, target, blocked, w, h) -> Optional[List[Tuple[int, int]]]:
    """BFS tìm đường đi ngắn nhất cụ thể [start, ..., target] cho reconstruction."""
    if start == target:
        return [start]
    if target in blocked:
        return None
    q = deque([start])
    vis = {start}
    prev = {start: None}
    while q:
        pos = q.popleft()
        for dx, dy in _DIRS:
            np_ = (pos[0] + dx, pos[1] + dy)
            if np_ in vis or np_ in blocked or not (0 <= np_[0] < w and 0 <= np_[1] < h):
                continue
            prev[np_] = pos
            if np_ == target:
                path = []
                c = np_
                while c is not None:
                    path.append(c)
                    c = prev[c]
                path.reverse()
                return path
            vis.add(np_)
            q.append(np_)
    return None


# ═════════════════════════════════════════════════════════════════════
#  2. TIỀN XỬ LÝ ĐỊA HÌNH & BẾ TẮC (MAP PREPROCESSING)
# ═════════════════════════════════════════════════════════════════════

class MapInfo:
    """Container lưu trữ dữ liệu tiền xử lý địa hình (chỉ tính 1 lần duy nhất cho mỗi map)."""
    def __init__(self, state):
        self.width = state.width
        self.height = state.height
        self.obstacles = state.obstacles
        self.storage = state.storage
        self.dead_squares = self._compute_dead_squares()
        self.dist_matrix = self._compute_distance_matrix()
        self.goal_corridors = self._compute_goal_corridors()

    def is_walkable(self, x, y) -> bool:
        return 0 <= x < self.width and 0 <= y < self.height and (x, y) not in self.obstacles

    def _compute_dead_squares(self) -> frozenset:
        """Reverse BFS (Pull Search) từ các ô đích tìm toàn bộ góc chết tĩnh."""
        alive = set(self.storage)
        queue = deque(self.storage)
        while queue:
            cx, cy = queue.popleft()
            for dx, dy in _DIRS:
                nx, ny = cx + dx, cy + dy
                px, py = cx + 2 * dx, cy + 2 * dy
                if self.is_walkable(nx, ny) and self.is_walkable(px, py):
                    if (nx, ny) not in alive:
                        alive.add((nx, ny))
                        queue.append((nx, ny))

        dead = set()
        for x in range(self.width):
            for y in range(self.height):
                if self.is_walkable(x, y) and (x, y) not in alive:
                    dead.add((x, y))
        return frozenset(dead)

    def _compute_distance_matrix(self) -> Dict[Tuple[int, int], Dict[Tuple[int, int], int]]:
        """BFS tính khoảng cách đi bộ thực tế quanh tường từ mỗi ô đích."""
        dist_matrix = {}
        for goal in self.storage:
            dist_map = {goal: 0}
            queue = deque([goal])
            while queue:
                cx, cy = queue.popleft()
                cdist = dist_map[(cx, cy)]
                for dx, dy in _DIRS:
                    nx, ny = cx + dx, cy + dy
                    if self.is_walkable(nx, ny) and (nx, ny) not in dist_map:
                        dist_map[(nx, ny)] = cdist + 1
                        queue.append((nx, ny))
            dist_matrix[goal] = dist_map
        return dist_matrix

    def _compute_goal_corridors(self) -> List[Tuple[Tuple[int, int], ...]]:
        """Phát hiện các chuỗi đích nằm trong ngõ cụt để bắt buộc lấp Inside-Out."""
        def get_nbrs(x, y):
            return [(x + dx, y + dy) for dx, dy in _DIRS if self.is_walkable(x + dx, y + dy)]

        chains = []
        visited = set()
        dead_ends = [
            (x, y) for x in range(self.width) for y in range(self.height)
            if self.is_walkable(x, y) and len(get_nbrs(x, y)) == 1
        ]
        storage_set = set(self.storage)
        for start in dead_ends:
            if start in visited:
                continue
            chain_cells = [start]
            visited.add(start)
            nbrs = get_nbrs(start[0], start[1])
            if not nbrs:
                continue
            prev, curr = start, nbrs[0]
            while curr not in visited:
                curr_nbrs = get_nbrs(curr[0], curr[1])
                if len(curr_nbrs) == 2:
                    chain_cells.append(curr)
                    visited.add(curr)
                    nxt = curr_nbrs[0] if curr_nbrs[1] == prev else curr_nbrs[1]
                    prev, curr = curr, nxt
                else:
                    break
            goals = [c for c in chain_cells if c in storage_set]
            if len(goals) >= 2:
                chains.append(tuple(goals))
        return chains

    def is_corridor_goal_blocked(self, boxes: frozenset) -> bool:
        """Kiểm tra có thùng nào bít lối vào ô đích sâu hơn trong hành lang cụt không."""
        for chain in self.goal_corridors:
            inner_empty = False
            for goal in chain:
                if goal not in boxes:
                    inner_empty = True
                elif inner_empty:
                    return True
        return False


_MAP_INFO_CACHE: Dict[Tuple, MapInfo] = {}

def get_map_info(state) -> MapInfo:
    key = (state.width, state.height, state.obstacles, state.storage)
    if key not in _MAP_INFO_CACHE:
        _MAP_INFO_CACHE[key] = MapInfo(state)
    return _MAP_INFO_CACHE[key]


# ═════════════════════════════════════════════════════════════════════
#  3. PHÁT HIỆN BẾ TẮC (DEADLOCK DETECTION)
# ═════════════════════════════════════════════════════════════════════

def is_2x2_deadlock(state) -> bool:
    """Phát hiện khối 2x2 kẹt cứng gồm toàn tường và thùng."""
    boxes = state.boxes
    obstacles = state.obstacles
    storage = state.storage
    w, h = state.width, state.height
    unplaced = [b for b in boxes if b not in storage]
    if not unplaced:
        return False

    def is_solid(x, y):
        if x < 0 or x >= w or y < 0 or y >= h:
            return True
        return (x, y) in obstacles or (x, y) in boxes

    def check_2x2(x, y):
        if not (is_solid(x, y) and is_solid(x + 1, y) and is_solid(x, y + 1) and is_solid(x + 1, y + 1)):
            return False
        for cx, cy in ((x, y), (x + 1, y), (x, y + 1), (x + 1, y + 1)):
            if (cx, cy) in boxes and (cx, cy) not in storage:
                return True
        return False

    for bx, by in unplaced:
        if (check_2x2(bx, by) or check_2x2(bx - 1, by) or
            check_2x2(bx, by - 1) or check_2x2(bx - 1, by - 1)):
            return True
    return False

def is_deadlock(state, map_info: MapInfo) -> bool:
    """Kiểm tra tổng hợp mọi dạng bế tắc của trạng thái hiện tại."""
    # 1. Deadlock tĩnh: Thùng bị đẩy vào ô chết tĩnh
    for box in state.boxes:
        if box in map_info.dead_squares:
            return True
    # 2. Deadlock động 2x2
    if is_2x2_deadlock(state):
        return True
    # 3. Deadlock bít cửa hành lang cụt
    if map_info.is_corridor_goal_blocked(state.boxes):
        return True
    return False


# ═════════════════════════════════════════════════════════════════════
#  4. HEURISTIC TỔNG HỢP DẪN ĐƯỜNG MẠNH MẼ (UNIFIED HEURISTIC)
# ═════════════════════════════════════════════════════════════════════

_assign_cache = {}
_box_match_cache = {}

def _best_robot_assignment(robots, boxes):
    n_r, n_b = len(robots), len(boxes)
    k = min(n_r, n_b)
    if n_b <= _MAX_BOXES_FOR_DP:
        layer = {0: (0, ())}
        for r in robots:
            nxt = {}
            for mask, (c, pairs) in layer.items():
                if mask not in nxt or c < nxt[mask][0]:
                    nxt[mask] = (c, pairs)
                for j in range(n_b):
                    if not (mask >> j) & 1:
                        nm = mask | (1 << j)
                        nc = c + approach_cost(r, boxes[j])
                        if nm not in nxt or nc < nxt[nm][0]:
                            nxt[nm] = (nc, pairs + ((r, boxes[j]),))
            layer = nxt
        return min((v for m, v in layer.items() if bin(m).count("1") == k), key=lambda v: v[0])

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

def robot_to_box_heuristic(state) -> int:
    robots = tuple(sorted(state.robots))
    unplaced = tuple(sorted(b for b in state.boxes if b not in state.storage))
    if not unplaced or not robots:
        return 0
    key = (robots, unplaced)
    cached = _assign_cache.get(key)
    if cached is None:
        if len(_assign_cache) > 100000:
            _assign_cache.clear()
        cached = _best_robot_assignment(robots, unplaced)
        _assign_cache[key] = cached
    return cached[0]

def box_to_storage_heuristic(state, dist_matrix) -> float:
    cache_key = (state.boxes, state.storage, state.obstacles)
    cached = _box_match_cache.get(cache_key)
    if cached is not None:
        return cached

    boxes = sorted(state.boxes)
    goals = sorted(state.storage)
    if not boxes:
        return 0
    if len(goals) < len(boxes):
        return float("inf")

    pairs = []
    for i, box in enumerate(boxes):
        for j, goal in enumerate(goals):
            if dist_matrix and goal in dist_matrix:
                d = dist_matrix[goal].get(box, float("inf"))
            else:
                d = manhattan_distance(box, goal)
            pairs.append((d, i, j))
    pairs.sort()

    used_boxes = set()
    used_goals = set()
    total = 0
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

    res = float("inf") if len(used_boxes) < len(boxes) else total
    _box_match_cache[cache_key] = res
    return res

def unified_heuristic(state, map_info: MapInfo) -> float:
    h_box = box_to_storage_heuristic(state, map_info.dist_matrix)
    if h_box == float("inf"):
        return float("inf")
    h_robot = robot_to_box_heuristic(state)
    return h_box + h_robot


# ═════════════════════════════════════════════════════════════════════
#  5. MACRO PUSH & RECONSTRUCTION
# ═════════════════════════════════════════════════════════════════════

class _Node:
    __slots__ = ('state', 'f', 'h')
    def __init__(self, s, f, h):
        self.state = s
        self.f = f
        self.h = h
    def __lt__(self, other):
        if self.f != other.f:
            return self.f < other.f
        return self.h < other.h

def macro_successors(state, map_info: MapInfo):
    """Sinh tất cả trạng thái kế tiếp bằng Macro Push (1 lần đẩy thùng)."""
    dead_sq = map_info.dead_squares
    w, h = state.width, state.height
    obs = state.obstacles
    boxes = state.boxes
    robots = state.robots
    nr = len(robots)
    storage = state.storage
    robot_set = frozenset(robots)

    reach_strict = []
    for r in range(nr):
        other_r = frozenset(robots[j] for j in range(nr) if j != r)
        reach_strict.append(_flood(robots[r], obs | boxes | other_r, w, h))

    reach_relaxed = []
    blocked_base = obs | boxes
    for r in range(nr):
        reach_relaxed.append(_flood(robots[r], blocked_base, w, h))

    strict_moves = []
    relaxed_moves = []

    for box in boxes:
        bx, by = box
        for dx, dy in _DIRS:
            nb = (bx + dx, by + dy)
            if not (0 <= nb[0] < w and 0 <= nb[1] < h):
                continue
            if nb in obs or nb in boxes or nb in dead_sq:
                continue

            pp = (bx - dx, by - dy)
            if not (0 <= pp[0] < w and 0 <= pp[1] < h):
                continue
            if pp in obs or pp in boxes:
                continue

            found_strict = False
            for r in range(nr):
                if nb in robot_set and nb != robots[r]:
                    continue
                if pp in reach_strict[r]:
                    found_strict = True
                    walk_cost = reach_strict[r][pp]
                    total_cost = walk_cost + 1

                    new_robots = list(robots)
                    new_robots[r] = box
                    new_boxes = frozenset((boxes - {box}) | {nb})

                    ns = SokobanState(
                        f"{r} push_{_DIR_NAME[(dx, dy)]}",
                        state.gval + total_cost,
                        state,
                        w, h, tuple(new_robots), new_boxes, storage, obs
                    )
                    strict_moves.append((ns, total_cost))

            if not found_strict:
                best_cost = float('inf')
                best_r = -1
                for r in range(nr):
                    if nb in robot_set and nb != robots[r]:
                        continue
                    if pp in reach_relaxed[r]:
                        ideal_dist = reach_relaxed[r][pp]
                        blocking_est = sum(
                            1 for j in range(nr)
                            if j != r and robots[j] in reach_relaxed[r] and reach_relaxed[r][robots[j]] <= ideal_dist
                        )
                        est_cost = ideal_dist + 2 * blocking_est
                        if est_cost < best_cost:
                            best_cost = est_cost
                            best_r = r

                if best_r != -1:
                    total_cost = best_cost + 1
                    new_robots = list(robots)
                    new_robots[best_r] = box
                    new_boxes = frozenset((boxes - {box}) | {nb})

                    ns = SokobanState(
                        f"{best_r} push_{_DIR_NAME[(dx, dy)]}",
                        state.gval + total_cost,
                        state,
                        w, h, tuple(new_robots), new_boxes, storage, obs
                    )
                    relaxed_moves.append((ns, total_cost))

    if strict_moves:
        return strict_moves
    return relaxed_moves

def _step_robot(current, robot_idx, new_pos, w, h):
    dx = new_pos[0] - current.robots[robot_idx][0]
    dy = new_pos[1] - current.robots[robot_idx][1]
    new_robots = list(current.robots)
    new_robots[robot_idx] = new_pos
    return SokobanState(
        f"{robot_idx} {_DIR_NAME[(dx, dy)]}",
        current.gval + 1,
        current,
        w, h, tuple(new_robots), current.boxes, current.storage, current.obstacles
    )

def _walk_robot_to(current, robot_idx, target, w, h):
    robots = current.robots
    nr = len(robots)
    if robots[robot_idx] == target:
        return current, True

    obs = current.obstacles
    boxes = current.boxes
    other_r = frozenset(robots[j] for j in range(nr) if j != robot_idx)

    path = _bfs_path(robots[robot_idx], target, obs | boxes | other_r, w, h)
    if path is not None:
        cur = current
        for i in range(1, len(path)):
            cur = _step_robot(cur, robot_idx, path[i], w, h)
        return cur, True

    ideal = _bfs_path(robots[robot_idx], target, obs | boxes, w, h)
    if ideal is None:
        return current, False

    ideal_cells = set(ideal[1:])
    blockers = [(j, robots[j]) for j in range(nr) if j != robot_idx and robots[j] in ideal_cells]
    cur = current

    for b_idx, _ in blockers:
        b_pos = cur.robots[b_idx]
        if b_pos not in ideal_cells:
            continue
        cur_occupied = frozenset(cur.robots) | cur.boxes | obs
        q2 = deque([b_pos])
        vis2 = {b_pos}
        free_target = None
        while q2:
            p = q2.popleft()
            for ddx, ddy in _DIRS:
                np2 = (p[0] + ddx, p[1] + ddy)
                if np2 in vis2 or not (0 <= np2[0] < w and 0 <= np2[1] < h):
                    continue
                vis2.add(np2)
                if np2 not in cur_occupied:
                    if np2 not in ideal_cells:
                        free_target = np2
                        break
                    q2.append(np2)
            if free_target:
                break

        if free_target is None:
            continue
        other_b = frozenset(cur.robots[j] for j in range(nr) if j != b_idx)
        clear_path = _bfs_path(cur.robots[b_idx], free_target, obs | cur.boxes | other_b, w, h)
        if clear_path is None:
            continue
        for i in range(1, len(clear_path)):
            cur = _step_robot(cur, b_idx, clear_path[i], w, h)

    other_r2 = frozenset(cur.robots[j] for j in range(nr) if j != robot_idx)
    path2 = _bfs_path(cur.robots[robot_idx], target, obs | cur.boxes | other_r2, w, h)
    if path2 is None:
        return current, False

    for i in range(1, len(path2)):
        cur = _step_robot(cur, robot_idx, path2[i], w, h)
    return cur, True

def reconstruct(initial_state, macro_goal):
    chain = []
    cur = macro_goal
    while cur is not None:
        chain.append(cur)
        cur = cur.parent
    chain.reverse()

    if len(chain) <= 1:
        return initial_state

    w, h = initial_state.width, initial_state.height
    current = SokobanState(
        "START", 0, None,
        w, h, chain[0].robots, chain[0].boxes, chain[0].storage, chain[0].obstacles
    )

    for k in range(1, len(chain)):
        prev_m = chain[k - 1]
        next_m = chain[k]
        removed = prev_m.boxes - next_m.boxes
        added = next_m.boxes - prev_m.boxes
        if not removed or not added:
            continue

        old_box = next(iter(removed))
        new_box = next(iter(added))
        dx = new_box[0] - old_box[0]
        dy = new_box[1] - old_box[1]
        push_pos = (old_box[0] - dx, old_box[1] - dy)

        push_r = None
        for r in range(len(next_m.robots)):
            if next_m.robots[r] == old_box:
                push_r = r
                break
        if push_r is None:
            return False

        current, ok = _walk_robot_to(current, push_r, push_pos, w, h)
        if not ok:
            return False

        new_robots = list(current.robots)
        new_robots[push_r] = old_box
        new_boxes = frozenset((current.boxes - {old_box}) | {new_box})
        current = SokobanState(
            f"{push_r} {_DIR_NAME[(dx, dy)]}",
            current.gval + 1,
            current,
            w, h, tuple(new_robots), new_boxes, current.storage, current.obstacles
        )

    return current


# ═════════════════════════════════════════════════════════════════════
#  6. CÁC THUẬT TOÁN TÌM KIẾM (SEARCH ENGINES)
# ═════════════════════════════════════════════════════════════════════

def macro_astar(initial_state, w=2.0, timebound=120, map_info=None):
    """Bộ máy tìm kiếm chính: Weighted A* trên không gian Macro Push."""
    start_time = time.time()
    if map_info is None:
        map_info = get_map_info(initial_state)

    if is_deadlock(initial_state, map_info):
        return False

    h0 = unified_heuristic(initial_state, map_info)
    if h0 == float('inf'):
        return False

    pq = [_Node(initial_state, initial_state.gval + w * h0, h0)]
    explored = {initial_state.hashable_state(): initial_state.gval}

    while pq:
        if time.time() - start_time > timebound - 1.5:
            return False

        node = heapq.heappop(pq)
        st = node.state
        key = st.hashable_state()

        if st.gval > explored.get(key, float('inf')):
            continue

        if sokoban_goal_state(st):
            return reconstruct(initial_state, st)

        for ns, _ in macro_successors(st, map_info):
            nk = ns.hashable_state()
            ng = ns.gval

            if nk in explored and ng >= explored[nk]:
                continue

            if is_deadlock(ns, map_info):
                continue

            hv = unified_heuristic(ns, map_info)
            if hv == float('inf'):
                continue

            explored[nk] = ng
            heapq.heappush(pq, _Node(ns, ng + w * hv, hv))

    return False

def micro_astar(initial_state, w=2.0, timebound=120, map_info=None):
    """Bộ máy dự phòng an toàn: Weighted A* trên từng bước vi mô."""
    start_time = time.time()
    if map_info is None:
        map_info = get_map_info(initial_state)

    if is_deadlock(initial_state, map_info):
        return False

    h0 = unified_heuristic(initial_state, map_info)
    if h0 == float('inf'):
        return False

    pq = [_Node(initial_state, initial_state.gval + w * h0, h0)]
    explored = {initial_state.hashable_state(): initial_state.gval}

    while pq:
        if time.time() - start_time > timebound - 0.5:
            return False

        node = heapq.heappop(pq)
        st = node.state
        key = st.hashable_state()

        if st.gval > explored.get(key, float('inf')):
            continue

        if sokoban_goal_state(st):
            return st

        for ns in st.successors():
            nk = ns.hashable_state()
            ng = ns.gval

            if nk in explored and ng >= explored[nk]:
                continue

            if is_deadlock(ns, map_info):
                continue

            hv = unified_heuristic(ns, map_info)
            if hv == float('inf'):
                continue

            explored[nk] = ng
            heapq.heappush(pq, _Node(ns, ng + w * hv, hv))

    return False


# ═════════════════════════════════════════════════════════════════════
#  7. ENTRY POINT THEO CHUẨN ĐỀ BÀI (SOLVE)
# ═════════════════════════════════════════════════════════════════════

def check_early_stop(state, map_info: MapInfo):
    """Kiểm tra dừng sớm cho các trường hợp đặc biệt / vô nghiệm."""
    if sokoban_goal_state(state):
        return state
    if len(state.boxes) > len(state.storage):
        return False
    if not state.robots:
        return False
    if is_deadlock(state, map_info):
        return False
    return None

def solve(initial_state, timebound=120, w=2.0):
    """
    Hàm entry point chuẩn của sinh viên theo yêu cầu đề bài.
    Đầu vào: SokobanState ban đầu, timebound tính bằng giây, hệ số w (mặc định 2.0).
    Đầu ra: SokobanState đích với chuỗi parent hợp lệ, hoặc False.
    """
    if timebound <= 0:
        return False

    start_time = time.time()
    map_info = get_map_info(initial_state)

    # 1. Kiểm tra dừng sớm (Edge case)
    early = check_early_stop(initial_state, map_info)
    if early is not None:
        return early

    # 2. Bộ máy chính: Macro Push A* (chiếm 85% ngân sách thời gian)
    macro_budget = max(1.0, timebound * 0.85)
    result = macro_astar(initial_state, w=w, timebound=macro_budget, map_info=map_info)
    if result and result is not False:
        return result

    # 3. Bộ máy dự phòng: Micro A* (dùng thời gian còn lại nếu macro gặp khó)
    remaining_time = timebound - (time.time() - start_time)
    if remaining_time > 1.0:
        fallback_res = micro_astar(initial_state, w=w, timebound=remaining_time, map_info=map_info)
        if fallback_res and fallback_res is not False:
            return fallback_res

    return False
