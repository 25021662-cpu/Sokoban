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

    def is_corridor_goal_blocked(self, boxes: frozenset) -> bool:
        """Kiểm tra có thùng nào ở cửa hành lang bít lối vào ô đích sâu hơn còn trống không."""
        for chain in self.goal_corridors:
            inner_empty = False
            for goal in chain:
                if goal not in boxes:
                    inner_empty = True
                elif inner_empty:
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

def is_2x2_deadlock(state) -> bool:
    """
    Phát hiện các khối 2x2 bị đóng băng (Freeze Deadlock).
    Nếu 4 ô tạo thành hình vuông 2x2 chỉ gồm Tường và Thùng,
    thì không thùng nào trong khối đó có thể di chuyển được nữa.
    Nếu có ít nhất 1 thùng trong khối đó không nằm trên đích -> Deadlock vĩnh viễn.
    """
    boxes = state.boxes
    obstacles = state.obstacles
    storage = state.storage
    width = state.width
    height = state.height

    unplaced_boxes = [b for b in boxes if b not in storage]
    if not unplaced_boxes:
        return False

    def is_solid(x, y):
        if x < 0 or x >= width or y < 0 or y >= height:
            return True
        return (x, y) in obstacles or (x, y) in boxes

    def check_2x2(x, y):
        if not (is_solid(x, y) and is_solid(x + 1, y) and is_solid(x, y + 1) and is_solid(x + 1, y + 1)):
            return False
        for cx, cy in ((x, y), (x + 1, y), (x, y + 1), (x + 1, y + 1)):
            if (cx, cy) in boxes and (cx, cy) not in storage:
                return True
        return False

    for bx, by in unplaced_boxes:
        if (check_2x2(bx, by) or
            check_2x2(bx - 1, by) or
            check_2x2(bx, by - 1) or
            check_2x2(bx - 1, by - 1)):
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
    for box in state.boxes:
        if box in dead_squares:
            return True

    # 2. Deadlock động: Khối kẹt 2x2 (Freeze Deadlock)
    if is_2x2_deadlock(state):
        return True

    # 3. Deadlock phong tỏa ô đích (Blocked Goal Deadlock)
    if is_goal_blocked(state, dead_squares):
        return True

    # 4. Deadlock bít cửa hành lang/ngõ cụt (Corridor Goal Deadlock)
    if map_info is not None and map_info.is_corridor_goal_blocked(state.boxes):
        return True

    return False
