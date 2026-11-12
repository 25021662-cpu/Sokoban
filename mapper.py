"""
mapper.py - Module tích hợp Phân tích Bản đồ (Topological Mapping) & Phát hiện Bế tắc (Deadlock Pruning).

Bao gồm 2 khối chức năng chính:
1. KHỐI TIỀN XỬ LÝ ĐỊA HÌNH (Chạy 1 lần duy nhất cho mỗi map và lưu cache):
   - Phân loại hình thái không gian: DeadEnd, Corridor, Junction, Room.
   - Chuỗi thứ tự lấp đích (Goal Corridors): Xác định thứ tự Inside-Out trong hành lang cụt.
   - Ma trận khoảng cách thực tế (BFS Distance Transform): Tính số bước đi bộ ngắn nhất quanh tường.
   - Container MapInfo & hàm get_map_info(state).

2. KHỐI PHÁT HIỆN BẾ TẮC (Chạy trong vòng lặp tìm kiếm để cắt tỉa trạng thái):
   - Deadlock Tĩnh (Dead Squares): Thuật toán Reverse BFS (Pull Search) từ các ô đích.
   - Freeze Deadlock 2x2: Khối 2x2 kẹt cứng gồm toàn Tường/Thùng có ít nhất 1 thùng ngoài đích.
   - Corridor Goal Deadlock: Thùng ở cửa hành lang cụt bít lối vào các ô đích sâu hơn còn trống.
   - Blocked Goal Deadlock: Ô đích bị phong tỏa hoàn toàn.
   - Hàm kiểm tra tổng hợp: `is_deadlock(state, map_info=None)`.
"""

from collections import deque
from typing import Dict, List, Tuple, Set, Optional, Union

# ==============================================================================
# PHẦN 1: TIỀN XỬ LÝ ĐỊA HÌNH BẢN ĐỒ (TOPOLOGICAL MAPPING & PREPROCESSING)
# ==============================================================================

class NodeType:
    """Các kiểu hình thái ô sàn trên bản đồ."""
    DEAD_END = "DeadEnd"    # Ô ngõ cụt (1 hướng đi)
    CORRIDOR = "Corridor"   # Hành lang hẹp (2 hướng đi)
    JUNCTION = "Junction"   # Giao lộ / ngã ba / ngã tư (>= 3 hướng đi)
    ROOM = "Room"           # Vùng không gian mở


# Cache lưu trữ dữ liệu tiền xử lý theo từng bản đồ:
# Key: (width, height, obstacles_frozenset, storage_frozenset) -> Value
_DEAD_SQUARES_CACHE: Dict[Tuple, frozenset] = {}
_TOPOLOGY_CACHE: Dict[Tuple, Dict] = {}
_GOAL_CORRIDORS_CACHE: Dict[Tuple, List[Tuple[Tuple[int, int], ...]]] = {}
_DISTANCE_MATRIX_CACHE: Dict[Tuple, Dict[Tuple[int, int], Dict[Tuple[int, int], int]]] = {}
_MAP_INFO_CACHE: Dict[Tuple, "MapInfo"] = {}

_DIRECTIONS = ((0, -1), (0, 1), (-1, 0), (1, 0))


def compute_dead_squares(state) -> frozenset:
    """
    Tìm tất cả các ô Deadlock Tĩnh (Dead Squares) trên bản đồ bằng thuật toán Reverse BFS (Pull Search).
    Bao gồm mọi góc chết (Corner), cạnh chết (Edge), ngõ cụt và các hốc tường ziczac.
    Đảm bảo 0% False Positive (không bao giờ bắt nhầm ô hợp lệ).
    """
    cache_key = (state.width, state.height, state.obstacles, state.storage)
    if cache_key in _DEAD_SQUARES_CACHE:
        return _DEAD_SQUARES_CACHE[cache_key]

    width, height = state.width, state.height
    obstacles = state.obstacles
    storage = state.storage

    def is_walkable(x, y):
        return 0 <= x < width and 0 <= y < height and (x, y) not in obstacles

    alive = set(storage)
    queue = deque(storage)

    while queue:
        cx, cy = queue.popleft()
        for dx, dy in _DIRECTIONS:
            nx, ny = cx + dx, cy + dy          # vị trí thùng trước khi kéo vào (cx, cy)
            px, py = cx + 2 * dx, cy + 2 * dy  # vị trí robot đứng kéo

            if is_walkable(nx, ny) and is_walkable(px, py):
                if (nx, ny) not in alive:
                    alive.add((nx, ny))
                    queue.append((nx, ny))

    dead_squares = set()
    for x in range(width):
        for y in range(height):
            if is_walkable(x, y) and (x, y) not in alive:
                dead_squares.add((x, y))

    res = frozenset(dead_squares)
    _DEAD_SQUARES_CACHE[cache_key] = res
    return res


# Alias tương thích ngược với code cũ nếu có
map_err = compute_dead_squares


def analyze_topology(state) -> Dict[Tuple[int, int], str]:
    """Phân loại hình thái không gian của từng ô sàn trên bản đồ."""
    cache_key = (state.width, state.height, state.obstacles, state.storage)
    if cache_key in _TOPOLOGY_CACHE:
        return _TOPOLOGY_CACHE[cache_key]

    width, height = state.width, state.height
    obstacles = state.obstacles

    def is_walkable(x, y):
        return 0 <= x < width and 0 <= y < height and (x, y) not in obstacles

    topo = {}
    for x in range(width):
        for y in range(height):
            if not is_walkable(x, y):
                continue
            nbr_count = sum(1 for dx, dy in _DIRECTIONS if is_walkable(x + dx, y + dy))
            if nbr_count <= 1:
                topo[(x, y)] = NodeType.DEAD_END
            elif nbr_count == 2:
                topo[(x, y)] = NodeType.CORRIDOR
            else:
                topo[(x, y)] = NodeType.JUNCTION

    _TOPOLOGY_CACHE[cache_key] = topo
    return topo


def compute_goal_corridors(state) -> List[Tuple[Tuple[int, int], ...]]:
    """
    Phát hiện các chuỗi ô đích nằm trong hành lang hẹp dẫn vào ngõ cụt.
    Các ô đích trong chuỗi BẮT BUỘC phải lấp từ trong cùng ra ngoài (Inside-Out).
    Trả về: [(G_sâu_nhất, G_kế_tiếp, ..., G_ngoài_cùng), ...]
    """
    cache_key = (state.width, state.height, state.obstacles, state.storage)
    if cache_key in _GOAL_CORRIDORS_CACHE:
        return _GOAL_CORRIDORS_CACHE[cache_key]

    width, height = state.width, state.height
    obstacles = state.obstacles
    storage = set(state.storage)

    def is_walkable(x, y):
        return 0 <= x < width and 0 <= y < height and (x, y) not in obstacles

    def get_nbrs(x, y):
        return [(x + dx, y + dy) for dx, dy in _DIRECTIONS if is_walkable(x + dx, y + dy)]

    corridor_chains = []
    visited_cells = set()

    # Tìm các ô ngõ cụt (DeadEnd) làm điểm xuất phát (đáy hành lang)
    dead_ends = [
        (x, y) for x in range(width) for y in range(height)
        if is_walkable(x, y) and len(get_nbrs(x, y)) == 1
    ]

    for start in dead_ends:
        if start in visited_cells:
            continue

        chain_cells = [start]
        visited_cells.add(start)
        nbrs = get_nbrs(start[0], start[1])
        if not nbrs:
            continue

        prev = start
        curr = nbrs[0]

        # Lần theo hành lang (các ô có đúng 2 láng giềng đi được)
        while curr not in visited_cells:
            curr_nbrs = get_nbrs(curr[0], curr[1])
            if len(curr_nbrs) == 2:
                chain_cells.append(curr)
                visited_cells.add(curr)
                nxt = curr_nbrs[0] if curr_nbrs[1] == prev else curr_nbrs[1]
                prev, curr = curr, nxt
            else:
                break

        goals_in_chain = [cell for cell in chain_cells if cell in storage]
        if len(goals_in_chain) >= 2:
            corridor_chains.append(tuple(goals_in_chain))

    _GOAL_CORRIDORS_CACHE[cache_key] = corridor_chains
    return corridor_chains


def compute_distance_matrix(state) -> Dict[Tuple[int, int], Dict[Tuple[int, int], int]]:
    """
    Tính toán khoảng cách đi bộ thực tế quanh vật cản (True BFS Distance)
    từ mỗi ô đích đến tất cả các ô sàn đi được trên bản đồ.
    Trả về: Dict[goal, Dict[(x, y), distance]]
    """
    cache_key = (state.width, state.height, state.obstacles, state.storage)
    if cache_key in _DISTANCE_MATRIX_CACHE:
        return _DISTANCE_MATRIX_CACHE[cache_key]

    width, height = state.width, state.height
    obstacles = state.obstacles
    storage = state.storage

    def is_walkable(x, y):
        return 0 <= x < width and 0 <= y < height and (x, y) not in obstacles

    dist_matrix = {}
    for goal in storage:
        dist_map = {goal: 0}
        queue = deque([goal])

        while queue:
            cx, cy = queue.popleft()
            cdist = dist_map[(cx, cy)]
            for dx, dy in _DIRECTIONS:
                nx, ny = cx + dx, cy + dy
                if is_walkable(nx, ny) and (nx, ny) not in dist_map:
                    dist_map[(nx, ny)] = cdist + 1
                    queue.append((nx, ny))

        dist_matrix[goal] = dist_map

    _DISTANCE_MATRIX_CACHE[cache_key] = dist_matrix
    return dist_matrix


def get_true_distance(p1: Tuple[int, int], p2: Tuple[int, int], state, dist_matrix=None) -> int:
    """Tra cứu khoảng cách thực tế giữa 2 điểm quanh tường (O(1) nếu p2 hoặc p1 là đích)."""
    if dist_matrix is not None:
        if p2 in dist_matrix and p1 in dist_matrix[p2]:
            return dist_matrix[p2][p1]
        if p1 in dist_matrix and p2 in dist_matrix[p1]:
            return dist_matrix[p1][p2]
    return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])


def compute_constrained_lines(state) -> Tuple[Dict[int, int], Dict[int, int]]:
    """Phát hiện các hàng/cột sát biên bị giới hạn khả năng đẩy ra ngoài để kiểm tra sức chứa."""
    width, height = state.width, state.height
    obstacles = state.obstacles
    storage = state.storage

    def is_walkable(x, y):
        return 0 <= x < width and 0 <= y < height and (x, y) not in obstacles

    rows = {}
    for y in range(height):
        floor_xs = [x for x in range(width) if is_walkable(x, y)]
        if not floor_xs:
            continue
        all_up_blocked = all(not is_walkable(x, y - 1) for x in floor_xs)
        all_down_blocked = all(not is_walkable(x, y + 1) for x in floor_xs)
        if all_up_blocked or all_down_blocked:
            rows[y] = sum(1 for (gx, gy) in storage if gy == y)

    cols = {}
    for x in range(width):
        floor_ys = [y for y in range(height) if is_walkable(x, y)]
        if not floor_ys:
            continue
        all_left_blocked = all(not is_walkable(x - 1, y) for y in floor_ys)
        all_right_blocked = all(not is_walkable(x + 1, y) for y in floor_ys)
        if all_left_blocked or all_right_blocked:
            cols[x] = sum(1 for (gx, gy) in storage if gx == x)

    return rows, cols


def compute_tunnels(state) -> Tuple[frozenset, frozenset]:
    """Xác định các ô thuộc hành lang 1 ô (tunnel) theo trục ngang và trục dọc."""
    width, height = state.width, state.height
    obstacles = state.obstacles

    def is_walkable(x, y):
        return 0 <= x < width and 0 <= y < height and (x, y) not in obstacles

    th, tv = set(), set()
    for x in range(width):
        for y in range(height):
            if not is_walkable(x, y):
                continue
            if not is_walkable(x, y - 1) and not is_walkable(x, y + 1):
                th.add((x, y))
            if not is_walkable(x - 1, y) and not is_walkable(x + 1, y):
                tv.add((x, y))
    return frozenset(th), frozenset(tv)


class MapInfo:
    """Container lưu trữ toàn bộ dữ liệu tiền xử lý địa hình bản đồ."""
    def __init__(self, state):
        self.width: int = state.width
        self.height: int = state.height
        self.obstacles: frozenset = state.obstacles
        self.storage: frozenset = state.storage
        self.dead_squares: frozenset = compute_dead_squares(state)
        self.topology: Dict[Tuple[int, int], str] = analyze_topology(state)
        self.goal_corridors: List[Tuple[Tuple[int, int], ...]] = compute_goal_corridors(state)
        self.distance_matrix: Dict[Tuple[int, int], Dict[Tuple[int, int], int]] = compute_distance_matrix(state)
        self.dist_matrix = self.distance_matrix
        self.constrained_rows, self.constrained_cols = compute_constrained_lines(state)
        self.tunnel_h, self.tunnel_v = compute_tunnels(state)

    def is_walkable(self, x: int, y: int) -> bool:
        return 0 <= x < self.width and 0 <= y < self.height and (x, y) not in self.obstacles

    def is_tunnel(self, pos: Tuple[int, int], dx, dy=None) -> bool:
        """Kiểm tra ô pos có phải là tunnel theo phương di chuyển hay không."""
        if dy is None:
            dx, dy = dx
        if dy == 0:
            return pos in self.tunnel_h
        else:
            return pos in self.tunnel_v

    def is_corridor_goal_blocked(self, boxes: frozenset) -> bool:
        """Kiểm tra có thùng nào bít lối vào ô đích sâu hơn trong hành lang cụt làm thiếu số đích không."""
        unplaced_boxes = sum(1 for b in boxes if b not in self.storage)
        if unplaced_boxes == 0:
            return False

        trapped_goals = set()
        for chain in self.goal_corridors:
            inner_empty = False
            empty_in_chain = set()
            for i, goal in enumerate(chain):
                if goal not in boxes:
                    inner_empty = True
                    empty_in_chain.add(goal)
                elif inner_empty:
                    if i > 0 and chain[i - 1] in boxes:
                        trapped_goals.update(empty_in_chain)

        if trapped_goals:
            empty_goals = set(self.storage) - set(boxes)
            if len(empty_goals - trapped_goals) < unplaced_boxes:
                return True
        return False

    def is_line_deadlock(self, boxes: frozenset) -> bool:
        """Kiểm tra số thùng vượt quá số đích trên các hàng/cột sát biên."""
        if self.constrained_rows:
            row_counts = {}
            for bx, by in boxes:
                if by in self.constrained_rows:
                    row_counts[by] = row_counts.get(by, 0) + 1
            for y, count in row_counts.items():
                if count > self.constrained_rows[y]:
                    return True
        if self.constrained_cols:
            col_counts = {}
            for bx, by in boxes:
                if bx in self.constrained_cols:
                    col_counts[bx] = col_counts.get(bx, 0) + 1
            for x, count in col_counts.items():
                if count > self.constrained_cols[x]:
                    return True
        return False


def get_map_info(state) -> MapInfo:
    """Lấy hoặc tính toán đối tượng MapInfo từ cache cho bản đồ hiện tại."""
    cache_key = (state.width, state.height, state.obstacles, state.storage)
    if cache_key not in _MAP_INFO_CACHE:
        _MAP_INFO_CACHE[cache_key] = MapInfo(state)
    return _MAP_INFO_CACHE[cache_key]


# ==============================================================================
# PHẦN 2: PHÁT HIỆN BẾ TẮC (DEADLOCK PRUNING)
# ==============================================================================

_DEADLOCK_CACHE: Dict[frozenset, bool] = {}

def is_solid(x: int, y: int, w: int, h: int, solid: set) -> bool:
    return x < 0 or x >= w or y < 0 or y >= h or (x, y) in solid

def is_2x2_deadlock(state) -> bool:
    """
    Phát hiện các khối 2x2 bị đóng băng (Freeze Deadlock).
    Nếu 4 ô tạo thành hình vuông 2x2 chỉ gồm Tường và Thùng,
    thì không thùng nào trong khối đó có thể di chuyển được nữa.
    Nếu có ít nhất 1 thùng trong khối đó không nằm trên đích -> Deadlock vĩnh viễn.
    """
    boxes = state.boxes
    storage = state.storage
    obstacles = state.obstacles
    w, h = state.width, state.height

    unplaced = [b for b in boxes if b not in storage]
    if not unplaced:
        return False

    solid = set(obstacles) | set(boxes)
    for bx, by in unplaced:
        for x, y in ((bx - 1, by - 1), (bx, by - 1), (bx - 1, by), (bx, by)):
            if (is_solid(x, y, w, h, solid) and
                is_solid(x + 1, y, w, h, solid) and
                is_solid(x, y + 1, w, h, solid) and
                is_solid(x + 1, y + 1, w, h, solid)):
                return True
    return False


def is_freeze_deadlock(state, map_info: Optional[MapInfo] = None) -> bool:
    """Phát hiện bế tắc đóng băng tổng quát (General Axis-Freeze Deadlock) & Bỏ đói ô đích (Starvation)."""
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

    # Starvation Deadlock: Ô đích không còn bất kỳ lối vào khả thi nào do tường hoặc thùng đóng băng bít kín
    empty_goals = set(storage) - set(boxes)
    if empty_goals:
        dead_goals = set()
        for g in empty_goals:
            can_enter = False
            gx, gy = g
            for dx, dy in _DIRECTIONS:
                entry = (gx + dx, gy + dy)
                pp = (gx + 2 * dx, gy + 2 * dy)
                if not is_solid(entry[0], entry[1], w, h, frozen) and not is_solid(pp[0], pp[1], w, h, frozen):
                    can_enter = True
                    break
            if not can_enter:
                dead_goals.add(g)

        if dead_goals:
            unplaced_boxes = len(unplaced)
            if len(empty_goals - dead_goals) < unplaced_boxes:
                return True

    return False


def is_box_frozen(bx: int, by: int, state) -> bool:
    """Kiểm tra xem thùng tại (bx, by) có bị kẹt cứng cả 2 trục hay không."""
    obstacles = state.obstacles
    boxes = state.boxes
    width = state.width
    height = state.height

    def is_blocked(x, y):
        if x < 0 or x >= width or y < 0 or y >= height:
            return True
        return (x, y) in obstacles or (x, y) in boxes

    horiz_free = not is_blocked(bx - 1, by) and not is_blocked(bx + 1, by)
    vert_free = not is_blocked(bx, by - 1) and not is_blocked(bx, by + 1)
    return not (horiz_free or vert_free)


def is_goal_blocked(state, dead_squares=None) -> bool:
    """Phát hiện ô đích bị phong tỏa hoặc số đích khả thi < số thùng cần vào đích."""
    if dead_squares is None:
        dead_squares = compute_dead_squares(state)

    unplaced_boxes = [b for b in state.boxes if b not in state.storage]
    if not unplaced_boxes:
        return False

    empty_goals = [s for s in state.storage if s not in state.boxes]
    if len(empty_goals) < len(unplaced_boxes):
        return True

    obstacles = state.obstacles
    boxes = state.boxes
    width = state.width
    height = state.height

    def is_walkable(x, y):
        return 0 <= x < width and 0 <= y < height and (x, y) not in obstacles

    viable_goals_count = 0
    for gx, gy in empty_goals:
        can_receive = False
        for dx, dy in _DIRECTIONS:
            nx, ny = gx + dx, gy + dy
            px, py = gx + 2 * dx, gy + 2 * dy

            if not is_walkable(nx, ny) or not is_walkable(px, py):
                continue
            if (nx, ny) in dead_squares:
                continue
            if (nx, ny) in boxes:
                if (px, py) in boxes and is_box_frozen(px, py, state):
                    continue
                if is_box_frozen(nx, ny, state):
                    continue
            can_receive = True
            break

        if can_receive:
            viable_goals_count += 1

    return viable_goals_count < len(unplaced_boxes)


def is_deadlock(state, map_info_or_deadlocks=None) -> bool:
    """
    Hàm tổng hợp duy nhất kiểm tra xem trạng thái hiện tại có bị bế tắc hay không.
    Hỗ trợ truyền vào đối tượng MapInfo (khuyến nghị) hoặc frozenset dead_squares (tương thích cũ).
    """
    boxes = state.boxes
    if boxes in _DEADLOCK_CACHE:
        return _DEADLOCK_CACHE[boxes]

    map_info = None
    dead_squares = None

    if isinstance(map_info_or_deadlocks, MapInfo):
        map_info = map_info_or_deadlocks
        dead_squares = map_info.dead_squares
    elif isinstance(map_info_or_deadlocks, (frozenset, set)):
        dead_squares = map_info_or_deadlocks
    elif map_info_or_deadlocks is None:
        map_info = get_map_info(state)
        dead_squares = map_info.dead_squares

    # 1. Deadlock tĩnh: Thùng bị đẩy vào ô chết tĩnh
    if dead_squares:
        for box in boxes:
            if box in dead_squares:
                _DEADLOCK_CACHE[boxes] = True
                return True

    # 2. Deadlock động: Khối kẹt 2x2 (Freeze Deadlock)
    if is_2x2_deadlock(state):
        _DEADLOCK_CACHE[boxes] = True
        return True

    # 3. Deadlock đóng băng lan truyền tổng quát và bỏ đói ô đích góc
    if is_freeze_deadlock(state, map_info):
        _DEADLOCK_CACHE[boxes] = True
        return True

    # 4. Deadlock bít cửa hành lang/ngõ cụt (Corridor Goal Deadlock)
    if map_info is not None and getattr(map_info, 'goal_corridors', None) and map_info.is_corridor_goal_blocked(boxes):
        _DEADLOCK_CACHE[boxes] = True
        return True

    # 5. Deadlock đường biên / tường
    if map_info is not None and (getattr(map_info, 'constrained_rows', None) or getattr(map_info, 'constrained_cols', None)):
        if map_info.is_line_deadlock(boxes):
            _DEADLOCK_CACHE[boxes] = True
            return True

    # 6. Deadlock phong tỏa ô đích
    if is_goal_blocked(state, dead_squares):
        _DEADLOCK_CACHE[boxes] = True
        return True

    _DEADLOCK_CACHE[boxes] = False
    return False
