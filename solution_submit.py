"""
solution_submit.py - Bộ giải Sokoban Macro Push Weighted A*.

Cấu trúc các thành phần chính trong file:
1. TIỀN XỬ LÝ ĐỊA HÌNH & BẾ TẮC (MapInfo):
   - Reverse BFS (Pull Search): Tìm tập ô chết tĩnh (Dead Squares) không thể đẩy thùng về bất kỳ đích nào.
   - Distance Matrix: Reverse BFS tính ma trận khoảng cách đẩy thùng thực tế quanh chướng ngại vật từ mỗi ô đích.
   - Goal Corridors: Xác định các hành lang cụt chứa chuỗi ô đích để kiểm soát thứ tự lấp đầy (Inside-Out).
   - Constrained Lines: Phát hiện các hàng/cột sát biên bị giới hạn khả năng đẩy ra ngoài để kiểm tra sức chứa.

2. PHÁT HIỆN BẾ TẮC ĐỘNG (Deadlock Detection):
   - Static Dead Squares: Thùng nằm trên ô chết tĩnh đã tiền xử lý.
   - 2x2 Deadlock: Cụm 4 ô vuông gồm toàn tường và thùng chưa hoàn thành.
   - General Axis-Freeze Deadlock: Lan truyền trạng thái bất động theo trục X và Y từ tường sang các thùng lân cận.
   - Goal Corridor Deadlock: Thùng ở đích phía ngoài chặn lối vào đích phía trong khi không thể đẩy tiếp.
   - Line Deadlock: Số thùng trên hàng/cột sát biên vượt quá số ô đích trên hàng/cột đó.

3. HEURISTIC DẪN ĐƯỜNG:
   - Box-to-Goal: Ghép cặp 1-1 tối ưu giữa Thùng và Đích bằng thuật toán Hungarian O(V^3) trên ma trận khoảng cách BFS thực tế.

4. BỘ MÁY TÌM KIẾM (Search Engine):
   - Macro Push Weighted A* (mặc định w=2.3, tự thích ứng w cho map đông robot) trên không gian trạng thái macro.
   - Reconstruction: Tái tạo chuỗi trạng thái Macro Push thành chuỗi bước vi mô chuẩn luật chơi Sokoban.
"""

import heapq
import time
from collections import deque
from typing import Dict, List, Tuple, Optional

from sokoban import SokobanState, sokoban_goal_state

# ═════════════════════════════════════════════════════════════════════
#  1. HẰNG SỐ & HÀM TIỆN ÍCH HÌNH HỌC / ĐỒ THỊ
# ═════════════════════════════════════════════════════════════════════

_DIRS = ((0, -1), (0, 1), (-1, 0), (1, 0))
_DIR_NAME = {(0, -1): "up", (0, 1): "down", (-1, 0): "left", (1, 0): "right"}

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
        self.constrained_rows, self.constrained_cols = self._compute_constrained_lines()
        self.tunnel_h, self.tunnel_v = self._compute_tunnels()
        self.corner_goals = self._compute_corner_goals()

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
        """Reverse Pull BFS tính khoảng cách đẩy thùng thực tế quanh tường từ mỗi ô đích."""
        dist_matrix = {}
        for goal in self.storage:
            dist_map = {goal: 0}
            queue = deque([goal])
            while queue:
                cx, cy = queue.popleft()
                cdist = dist_map[(cx, cy)]
                for dx, dy in _DIRS:
                    nx, ny = cx + dx, cy + dy
                    px, py = cx + 2 * dx, cy + 2 * dy
                    if self.is_walkable(nx, ny) and self.is_walkable(px, py):
                        if (nx, ny) not in dist_map:
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
        """Kiểm tra có thùng nào bít lối vào ô đích sâu hơn trong hành lang cụt không.

        Chỉ báo deadlock khi thùng tại đích ngoài THỰC SỰ bị kẹt, tức là đích liền kề
        sâu hơn (chain[i-1]) cũng đã có thùng. Nếu chain[i-1] còn trống, thùng tại
        chain[i] vẫn có thể đẩy vào trong → không phải deadlock.
        """
        for chain in self.goal_corridors:
            inner_empty = False
            for i, goal in enumerate(chain):
                if goal not in boxes:
                    inner_empty = True
                elif inner_empty:
                    # Chỉ deadlock khi đích sâu hơn liền kề cũng có thùng
                    # (thùng tại chain[i] thực sự không còn chỗ để đẩy sâu hơn)
                    if i > 0 and chain[i - 1] in boxes:
                        return True
        return False

    def _compute_constrained_lines(self) -> Tuple[Tuple[Tuple[int, int], ...], Tuple[Tuple[int, int], ...]]:
        """Phát hiện các hàng/cột có tường bao liên tục chặn hoàn toàn hướng đẩy ra ngoài."""
        w, h, obs, storage = self.width, self.height, self.obstacles, self.storage
        constrained_rows = []
        for y in range(h):
            walkable = [x for x in range(w) if (x, y) not in obs]
            if not walkable:
                continue
            top_solid = all(y - 1 < 0 or (x, y - 1) in obs for x in walkable)
            bot_solid = all(y + 1 >= h or (x, y + 1) in obs for x in walkable)
            if top_solid or bot_solid:
                g_cnt = sum(1 for g in storage if g[1] == y)
                constrained_rows.append((y, g_cnt))

        constrained_cols = []
        for x in range(w):
            walkable = [y for y in range(h) if (x, y) not in obs]
            if not walkable:
                continue
            left_solid = all(x - 1 < 0 or (x - 1, y) in obs for y in walkable)
            right_solid = all(x + 1 >= w or (x + 1, y) in obs for y in walkable)
            if left_solid or right_solid:
                g_cnt = sum(1 for g in storage if g[0] == x)
                constrained_cols.append((x, g_cnt))

        return tuple(constrained_rows), tuple(constrained_cols)

    def is_line_deadlock(self, boxes: frozenset) -> bool:
        """Kiểm tra số thùng trên hàng/cột sát tường có vượt quá số đích có thể chứa không."""
        for y, max_g in self.constrained_rows:
            if sum(1 for b in boxes if b[1] == y) > max_g:
                return True
        for x, max_g in self.constrained_cols:
            if sum(1 for b in boxes if b[0] == x) > max_g:
                return True
        return False

    def _compute_tunnels(self) -> Tuple[frozenset, frozenset]:
        """Xác định các ô thuộc hành lang 1 ô (tunnel) theo trục ngang và trục dọc."""
        w, h = self.width, self.height
        th, tv = set(), set()
        for x in range(w):
            for y in range(h):
                if not self.is_walkable(x, y):
                    continue
                if not self.is_walkable(x, y - 1) and not self.is_walkable(x, y + 1):
                    th.add((x, y))
                if not self.is_walkable(x - 1, y) and not self.is_walkable(x + 1, y):
                    tv.add((x, y))
        return frozenset(th), frozenset(tv)

    def is_tunnel(self, pos: Tuple[int, int], dx: int, dy: int) -> bool:
        """Kiểm tra ô pos có nằm trong hành lang 1 ô theo hướng đẩy (dx, dy) hay không."""
        return (pos in self.tunnel_h) if dy == 0 else (pos in self.tunnel_v)

    def _compute_corner_goals(self) -> frozenset:
        """Xác định các ô đích nằm trong góc (có ít nhất 2 bức tường kề nhau tạo góc vuông)."""
        corners = set()
        for gx, gy in self.storage:
            top = not self.is_walkable(gx, gy - 1)
            bot = not self.is_walkable(gx, gy + 1)
            left = not self.is_walkable(gx - 1, gy)
            right = not self.is_walkable(gx + 1, gy)
            if (top and left) or (top and right) or (bot and left) or (bot and right):
                corners.add((gx, gy))
        return frozenset(corners)


_MAP_INFO_CACHE: Dict[Tuple, MapInfo] = {}

def get_map_info(state) -> MapInfo:
    key = (state.width, state.height, state.obstacles, state.storage)
    if key not in _MAP_INFO_CACHE:
        _MAP_INFO_CACHE[key] = MapInfo(state)
    return _MAP_INFO_CACHE[key]


# ═════════════════════════════════════════════════════════════════════
#  3. PHÁT HIỆN BẾ TẮC (DEADLOCK DETECTION)
# ═════════════════════════════════════════════════════════════════════

def is_solid(x: int, y: int, w: int, h: int, solid_set) -> bool:
    """Kiểm tra ô (x, y) có phải vật cản rắn (ngoài biên bản đồ hoặc nằm trong solid_set)."""
    return x < 0 or x >= w or y < 0 or y >= h or (x, y) in solid_set

def is_2x2_deadlock(state) -> bool:
    """Phát hiện khối 2x2 kẹt cứng gồm toàn tường và thùng."""
    boxes = state.boxes
    storage = state.storage
    unplaced = [b for b in boxes if b not in storage]
    if not unplaced:
        return False

    w, h = state.width, state.height
    solid = state.obstacles | boxes

    for bx, by in unplaced:
        for x, y in ((bx, by), (bx - 1, by), (bx, by - 1), (bx - 1, by - 1)):
            if (is_solid(x, y, w, h, solid) and
                is_solid(x + 1, y, w, h, solid) and
                is_solid(x, y + 1, w, h, solid) and
                is_solid(x + 1, y + 1, w, h, solid)):
                return True
    return False

def is_freeze_deadlock(state, map_info: Optional[MapInfo] = None) -> bool:
    """Phát hiện bế tắc đóng băng tổng quát (General Axis-Freeze Deadlock) & Bỏ đói ô đích (Starvation).

    Lan truyền trạng thái bất động theo 2 trục X và Y từ các bức tường:
    - Một thùng không thể di chuyển theo trục ngang (X) nếu ô bên trái hoặc bên phải là vật cản cứng (tường hoặc thùng đã bị đóng băng vĩnh viễn).
    - Tương tự, thùng không thể di chuyển theo trục dọc (Y) nếu ô bên trên hoặc bên dưới là vật cản cứng.
    - Một thùng bị khóa cả 2 trục X và Y sẽ trở thành bất động vĩnh viễn:
        + Nếu thùng này chưa nằm trên ô đích (storage) -> DEADLOCK ngay lập tức.
        + Nếu thùng này đã nằm trên ô đích -> thùng đóng băng và trở thành vật cản cứng cố định, có thể làm đóng băng dây chuyền các thùng lân cận.
    - Phát hiện Starvation Deadlock: Nếu một ô đích góc/nhạy cảm chưa có thùng, nhưng tất cả các hướng đẩy vào ô đó đều đã bị chặn bởi tường hoặc thùng đóng băng (frozen) -> DEADLOCK ngay lập tức.
    """
    boxes = state.boxes
    storage = state.storage
    obstacles = state.obstacles
    w, h = state.width, state.height

    unplaced = [b for b in boxes if b not in storage]
    if not unplaced:
        return False

    frozen = set(obstacles)
    changed = True
    while changed:
        changed = False
        for b in boxes:
            if b in frozen:
                continue
            x, y = b
            x_blocked = is_solid(x - 1, y, w, h, frozen) or is_solid(x + 1, y, w, h, frozen)
            y_blocked = is_solid(x, y - 1, w, h, frozen) or is_solid(x, y + 1, w, h, frozen)

            if x_blocked and y_blocked:
                if b not in storage:
                    return True
                frozen.add(b)
                changed = True

    # Starvation Deadlock: Ô đích góc không còn bất kỳ lối vào nào do tường hoặc thùng đóng băng bít kín
    if map_info and map_info.corner_goals:
        for g in map_info.corner_goals:
            if g not in boxes:
                can_enter = False
                gx, gy = g
                for dx, dy in _DIRS:
                    entry = (gx + dx, gy + dy)
                    pp = (gx + 2 * dx, gy + 2 * dy)
                    if not is_solid(entry[0], entry[1], w, h, frozen) and not is_solid(pp[0], pp[1], w, h, frozen):
                        can_enter = True
                        break
                if not can_enter:
                    return True

    return False

_DEADLOCK_CACHE: Dict[frozenset, bool] = {}

def is_deadlock(state, map_info: MapInfo) -> bool:
    """Kiểm tra tổng hợp các dạng bế tắc động với Cache O(1) theo cấu hình thùng (state.boxes)."""
    boxes = state.boxes
    if boxes in _DEADLOCK_CACHE:
        return _DEADLOCK_CACHE[boxes]

    # 1. Deadlock động 2x2 (kể cả khối 4 thùng lơ lửng giữa phòng)
    if is_2x2_deadlock(state):
        _DEADLOCK_CACHE[boxes] = True
        return True

    # 2. Deadlock đóng băng lan truyền tổng quát và bỏ đói ô đích góc
    if is_freeze_deadlock(state, map_info):
        _DEADLOCK_CACHE[boxes] = True
        return True

    # 3. Deadlock bít cửa hành lang cụt (chỉ chạy khi map có hành lang đích)
    if map_info.goal_corridors and map_info.is_corridor_goal_blocked(boxes):
        _DEADLOCK_CACHE[boxes] = True
        return True

    # 4. Deadlock đường biên / tường (chỉ chạy khi map có biên bị khóa)
    if (map_info.constrained_rows or map_info.constrained_cols) and map_info.is_line_deadlock(boxes):
        _DEADLOCK_CACHE[boxes] = True
        return True

    _DEADLOCK_CACHE[boxes] = False
    return False


# ═════════════════════════════════════════════════════════════════════
#  4. HEURISTIC DẪN ĐƯỜNG (HEURISTICS)
# ═════════════════════════════════════════════════════════════════════

_box_match_cache = {}

def clear_all_caches():
    """Xóa toàn bộ cache global sau mỗi lần giải xong 1 map."""
    _MAP_INFO_CACHE.clear()
    _box_match_cache.clear()
    _DEADLOCK_CACHE.clear()

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

    matching = []
    total_cost = 0
    for j in range(1, m + 1):
        if p[j] != 0:
            row = p[j] - 1
            col = j - 1
            matching.append((row, col))
            total_cost += cost_matrix[row][col]
    return total_cost, tuple(matching)

_UNREACHABLE_COST = 10**8

def box_to_storage_heuristic(state, dist_matrix) -> float:
    cache_key = state.boxes
    cached = _box_match_cache.get(cache_key)
    if cached is not None:
        return cached

    boxes = sorted(state.boxes)
    goals = sorted(state.storage)
    nb = len(boxes)
    ng = len(goals)
    if nb == 0:
        return 0
    if ng < nb:
        return float("inf")

    cost_matrix = [[dist_matrix[goal].get(box, _UNREACHABLE_COST) for goal in goals] for box in boxes]

    for r in range(nb):
        if all(cost_matrix[r][c] >= _UNREACHABLE_COST for c in range(ng)):
            _box_match_cache[cache_key] = float("inf")
            return float("inf")

    cost, pairs = min_cost_bipartite_matching(cost_matrix)
    if any(cost_matrix[r][c] >= _UNREACHABLE_COST for r, c in pairs):
        _box_match_cache[cache_key] = float("inf")
        return float("inf")

    _box_match_cache[cache_key] = cost
    return cost


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


def _local_may_split(c, in_region):
    """Kiểm tra cục bộ (vòng 8 ô): bỏ ô c khỏi vùng có THỂ làm tách vùng không."""
    x, y = c
    n, e, s, wv = (x, y - 1), (x + 1, y), (x, y + 1), (x - 1, y)
    ne, se, sw, nw = (x + 1, y - 1), (x + 1, y + 1), (x - 1, y + 1), (x - 1, y - 1)
    ring = [(n, ne, e), (e, se, s), (s, sw, wv), (wv, nw, n)]
    free4 = [p for p in (n, e, s, wv) if in_region(p)]
    if len(free4) <= 1:
        return False
    # union-find cục bộ trên 4 hướng, nối qua ô góc nếu trống
    parent = {p: p for p in free4}
    def find(p):
        while parent[p] != p:
            p = parent[p]
        return p
    for a, corner, b in ring:
        if in_region(a) and in_region(b) and in_region(corner):
            parent[find(a)] = find(b)
    return len({find(p) for p in free4}) > 1


def _relocation_options(j, pos, region, blocked_post, cap=3):
    """Các vị trí đại diện (cell, dist) mà robot j (đang rảnh) có thể đứng sau khi push làm tách vùng đi bộ."""
    cells = sorted((d, c) for c, d in region.items() if c not in blocked_post)
    seen = set()
    opts = []
    for d, c in cells:
        if c in seen:
            continue
        # BFS thành phần liên thông của c trong (region - blocked_post)
        comp = {c}
        dq = deque([c])
        while dq:
            cx, cy = dq.popleft()
            for dx, dy in _DIRS:
                q = (cx + dx, cy + dy)
                if q in comp or q not in region or q in blocked_post:
                    continue
                comp.add(q)
                dq.append(q)
        seen |= comp
        opts.append((c, d))          # c là ô gần j nhất trong thành phần (do đã sort theo dist)
        if len(opts) >= cap:
            break
    return opts if opts else [(pos, 0)]


def _evacuate_blockers(push_r, push_path, pp, robots, boxes, obs, w, h):
    """Tìm vị trí né đường an toàn cho các robot j != push_r đang đứng chắn trên push_path hoặc pp."""
    cleared_set = set(push_path) | {pp}
    nr = len(robots)
    blockers = [j for j in range(nr) if j != push_r and robots[j] in cleared_set]
    if not blockers:
        return list(robots), 0, True

    new_robots = list(robots)
    assigned_targets = set()
    static_robots = {robots[k] for k in range(nr) if k != push_r and k not in blockers}
    total_evac_cost = 0

    for b_idx in blockers:
        b_pos = robots[b_idx]
        q = deque([(b_pos, 0)])
        vis = {b_pos}
        target = None
        target_dist = 0

        while q:
            curr, d = q.popleft()
            if curr not in cleared_set and curr not in assigned_targets and curr not in static_robots:
                target = curr
                target_dist = d
                break

            for dx, dy in _DIRS:
                nxt = (curr[0] + dx, curr[1] + dy)
                if nxt in vis or not (0 <= nxt[0] < w and 0 <= nxt[1] < h):
                    continue
                if nxt in obs or nxt in boxes or nxt == robots[push_r]:
                    continue
                vis.add(nxt)
                q.append((nxt, d + 1))

        if target is None:
            return None, 0, False

        assigned_targets.add(target)
        new_robots[b_idx] = target
        total_evac_cost += target_dist

    return new_robots, total_evac_cost, True


def _idle_variants(robots, r, final_robot_pos, final_b, new_boxes, reach_strict, obs, push_path=(), pp=None):
    """Sinh các cấu hình vị trí robot rảnh sau 1 push của robot r.

    Mô hình macro chỉ cập nhật vị trí robot khi nó push, nên robot rảnh bị 'đóng băng' ở vị trí cũ.
    Nếu push của robot r chia đôi vùng đi bộ của robot rảnh j, j phải được chọn ở phía NÀO (trước khi push).
    Trả về list[(robots_tuple, extra_walk_cost)].
    """
    nr = len(robots)
    base = list(robots)
    base[r] = final_robot_pos
    per_robot = []
    reserved_push_cells = set(push_path) | ({pp} if pp is not None else set()) | {final_robot_pos}

    for j in range(nr):
        if j == r:
            continue
        region = reach_strict[j]
        if final_b not in region and final_robot_pos not in region:
            continue
        in_reg = lambda p, region=region: p in region and p != final_b and p != final_robot_pos
        if not ((final_b in region and _local_may_split(final_b, in_reg)) or
                (final_robot_pos in region and _local_may_split(final_robot_pos, in_reg))):
            continue
        blocked_post = set(obs) | set(new_boxes) | reserved_push_cells
        blocked_post.update(base[k] for k in range(nr) if k != j)
        opts = _relocation_options(j, robots[j], region, blocked_post)
        if len(opts) > 1:
            per_robot.append((j, opts))
    if not per_robot:
        return [(tuple(base), 0)]
    variants = [(base[:], 0)]
    for j, opts in per_robot:
        nv = []
        for cfg, cost in variants:
            for cell, d in opts:
                c2 = cfg[:]
                c2[j] = cell
                nv.append((c2, cost + d))
        variants = nv[:9]
    return [(tuple(c), k) for c, k in variants]


def macro_successors(state, map_info: MapInfo):
    """Sinh tất cả trạng thái kế tiếp bằng Macro Push (đẩy 1 bước hoặc đẩy thông luồng vào đích)."""
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
            nb1 = (bx + dx, by + dy)
            if not (0 <= nb1[0] < w and 0 <= nb1[1] < h):
                continue
            if nb1 in obs or nb1 in boxes or nb1 in dead_sq:
                continue

            pp = (bx - dx, by - dy)
            if not (0 <= pp[0] < w and 0 <= pp[1] < h):
                continue
            if pp in obs or pp in boxes:
                continue

            pushes = [(1, nb1)]

            # Ray-casting (Line Push & Tunnel Macro):
            # 1. Bắn tia tìm tất cả các ô đích (storage) dọc theo đường đẩy (tiếp tục qua đích trung gian để tìm đích sâu hơn).
            # 2. Nếu đi vào/qua hành lang hẹp 1 ô (tunnel), tự động đẩy xuyên suốt hành lang
            #    tới cửa thoát ra ngã rẽ / phòng lớn (Decision Point) trong 1 bước macro duy nhất.
            cur_b = nb1
            k = 1
            in_tunnel = map_info.is_tunnel(nb1, dx, dy)

            while True:
                # 1. Điểm dừng đích (Storage)
                if cur_b in storage:
                    if (k, cur_b) not in pushes:
                        pushes.append((k, cur_b))

                next_b = (cur_b[0] + dx, cur_b[1] + dy)
                if not (0 <= next_b[0] < w and 0 <= next_b[1] < h):
                    break
                if next_b in obs or next_b in boxes:
                    break
                # Chỉ dừng vì dead_sq nếu ô tiếp theo KHÔNG phải đích
                if next_b not in storage and next_b in dead_sq:
                    break

                # 2. Điểm thoát khỏi hành lang hẹp (Tunnel Exit / Decision Point)
                if in_tunnel and not map_info.is_tunnel(next_b, dx, dy):
                    if (k + 1, next_b) not in pushes:
                        pushes.append((k + 1, next_b))
                    in_tunnel = False

                k += 1
                cur_b = next_b

            for step_count, final_b in pushes:
                push_path = [(bx + i * dx, by + i * dy) for i in range(1, step_count + 1)]
                final_robot_pos = (final_b[0] - dx, final_b[1] - dy)
                eligible_strict = []
                for r in range(nr):
                    if pp in reach_strict[r]:
                        eligible_strict.append((reach_strict[r][pp], r))

                found_strict = False
                if eligible_strict:
                    chosen = sorted(eligible_strict)[:3]
                    for walk_cost, r in chosen:
                        evac_robots, evac_cost, ok = _evacuate_blockers(r, push_path, pp, robots, boxes, obs, w, h)
                        if not ok:
                            continue
                        found_strict = True
                        new_boxes = frozenset((boxes - {box}) | {final_b})
                        for new_robots, extra in _idle_variants(tuple(evac_robots), r, final_robot_pos, final_b,
                                                                new_boxes, reach_strict, obs, push_path, pp):
                            total_cost = walk_cost + step_count + evac_cost + extra
                            ns = SokobanState(
                                f"{r} push_{_DIR_NAME[(dx, dy)]} {step_count}",
                                state.gval + total_cost,
                                state,
                                w, h, new_robots, new_boxes, storage, obs
                            )
                            strict_moves.append((ns, total_cost))

                if not found_strict and step_count == 1:
                    best_cost = float('inf')
                    best_r = -1
                    best_evac = None
                    best_evac_cost = 0
                    for r in range(nr):
                        if pp in reach_relaxed[r]:
                            evac_robots, evac_cost, ok = _evacuate_blockers(r, push_path, pp, robots, boxes, obs, w, h)
                            if not ok:
                                continue
                            ideal_dist = reach_relaxed[r][pp]
                            blocking_est = sum(
                                1 for j in range(nr)
                                if j != r and robots[j] in reach_relaxed[r] and reach_relaxed[r][robots[j]] <= ideal_dist
                            )
                            est_cost = ideal_dist + 2 * blocking_est + evac_cost
                            if est_cost < best_cost:
                                best_cost = est_cost
                                best_r = r
                                best_evac = evac_robots
                                best_evac_cost = evac_cost

                    if best_r != -1:
                        total_cost = best_cost + 1
                        new_robots = list(best_evac)
                        new_robots[best_r] = final_robot_pos
                        new_boxes = frozenset((boxes - {box}) | {final_b})
                        ns = SokobanState(
                            f"{best_r} push_{_DIR_NAME[(dx, dy)]} 1",
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

        # Phase 2 (Fallback cho hành lang hẹp 1 ô): Nếu không có ngách rẽ ngoài ideal_cells,
        # tìm bất kỳ ô trống reachable nào mà khi blocker chuyển đến đó, robot chính có đường tới target
        if free_target is None:
            other_b_set = frozenset(cur.robots[j] for j in range(nr) if j != b_idx)
            q_corridor = deque([b_pos])
            vis_c = {b_pos}
            while q_corridor:
                cp = q_corridor.popleft()
                for ddx, ddy in _DIRS:
                    np_c = (cp[0] + ddx, cp[1] + ddy)
                    if np_c in vis_c or not (0 <= np_c[0] < w and 0 <= np_c[1] < h):
                        continue
                    if np_c in obs or np_c in cur.boxes or np_c in other_b_set:
                        continue
                    vis_c.add(np_c)
                    if np_c not in cur.robots:
                        test_other = frozenset(cur.robots[j] if j != b_idx else np_c for j in range(nr) if j != robot_idx)
                        if _bfs_path(cur.robots[robot_idx], target, obs | cur.boxes | test_other, w, h) is not None:
                            free_target = np_c
                            break
                    q_corridor.append(np_c)
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
    current = initial_state

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
        step_count = abs(dx) + abs(dy)
        udx = 0 if dx == 0 else (1 if dx > 0 else -1)
        udy = 0 if dy == 0 else (1 if dy > 0 else -1)

        push_pos = (old_box[0] - udx, old_box[1] - udy)
        target_robot_pos = (new_box[0] - udx, new_box[1] - udy)

        push_r = None
        for r in range(len(next_m.robots)):
            if next_m.robots[r] == target_robot_pos:
                push_r = r
                break
        if push_r is None:
            try:
                push_r = int(next_m.action.split()[0])
            except Exception:
                return False

        # Di chuyển các robot rảnh tới vị trí đã chọn TRƯỚC khi push (khi box còn chưa chia đôi vùng)
        for j in range(len(prev_m.robots)):
            if j != push_r and current.robots[j] != next_m.robots[j]:
                current, ok = _walk_robot_to(current, j, next_m.robots[j], w, h)
                if not ok:
                    return False

        current, ok = _walk_robot_to(current, push_r, push_pos, w, h)
        if not ok:
            return False

        curr_b = old_box
        for _ in range(step_count):
            next_b = (curr_b[0] + udx, curr_b[1] + udy)
            new_robots = list(current.robots)
            new_robots[push_r] = curr_b
            new_boxes = frozenset((current.boxes - {curr_b}) | {next_b})
            current = SokobanState(
                f"{push_r} {_DIR_NAME[(udx, udy)]}",
                current.gval + 1,
                current,
                w, h, tuple(new_robots), new_boxes, current.storage, current.obstacles
            )
            curr_b = next_b

    return current


# ═════════════════════════════════════════════════════════════════════
#  5. THUẬT TOÁN TÌM KIẾM (MACRO PUSH A*)
# ═════════════════════════════════════════════════════════════════════

def macro_astar(initial_state, w=2.0, timebound=120, map_info=None):
    """Bộ máy tìm kiếm chính: Weighted A* trên không gian Macro Push. Trả về SokobanState đích hoặc False."""
    start_time = time.time()
    if map_info is None:
        map_info = get_map_info(initial_state)

    if is_deadlock(initial_state, map_info):
        return False

    h0 = box_to_storage_heuristic(initial_state, map_info.dist_matrix)
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
            res = reconstruct(initial_state, st)
            if res and res is not False:
                return res
            # Nếu nhánh này gặp bế tắc khi reconstruct, tiếp tục thử các nhánh goal khác trong pq
            continue

        for ns, _ in macro_successors(st, map_info):
            nk = ns.hashable_state()
            ng = ns.gval

            if nk in explored and ng >= explored[nk]:
                continue

            if is_deadlock(ns, map_info):
                continue

            hv = box_to_storage_heuristic(ns, map_info.dist_matrix)
            if hv == float('inf'):
                continue

            explored[nk] = ng
            heapq.heappush(pq, _Node(ns, ng + w * hv, hv))

    return False


# ═════════════════════════════════════════════════════════════════════
#  6. ENTRY POINT THEO CHUẨN ĐỀ BÀI (SOLVE)
# ═════════════════════════════════════════════════════════════════════

def check_early_stop(state, map_info: MapInfo):
    """Kiểm tra dừng sớm cho các trường hợp đặc biệt / vô nghiệm (chặn map lỗi ban đầu)."""
    if sokoban_goal_state(state):
        return state
    # 1. Map lỗi: Số thùng > số đích
    if len(state.boxes) > len(state.storage):
        return False
    # 2. Map lỗi: Không có robot
    if not state.robots:
        return False
    # 3. Map lỗi: Thùng ban đầu nằm trên góc chết tĩnh
    for box in state.boxes:
        if box in map_info.dead_squares:
            return False
    # 4. Map lỗi: Bế tắc khởi đầu
    if is_deadlock(state, map_info):
        return False
    return None

def solve(initial_state, timebound=120, w=2.3):
    """
    Hàm entry point chuẩn của sinh viên theo yêu cầu đề bài.
    Đầu vào: SokobanState ban đầu, timebound tính bằng giây, hệ số w (mặc định 2.3).
    Đầu ra: SokobanState đích với chuỗi parent hợp lệ, hoặc False.
    """
    if timebound <= 0:
        return False
    
    clear_all_caches()

    map_info = get_map_info(initial_state)

    # 1. Kiểm tra dừng sớm (Edge case)
    early = check_early_stop(initial_state, map_info)
    if early is not None:
        return early

    # Đối với các bản đồ đông robot (Multi-Agent lớn > 4 robots), tăng hệ số w lên tối thiểu 2.8
    # để Weighted A* có định hướng tham lam sâu, tránh bùng nổ tổ hợp chiều rộng
    effective_w = max(w, 2.8) if len(initial_state.robots) > 4 else w

    # 2. Bộ máy tìm kiếm: Macro Push Weighted A* (hưởng trọn 100% ngân sách thời gian)
    return macro_astar(initial_state, w=effective_w, timebound=timebound, map_info=map_info)
