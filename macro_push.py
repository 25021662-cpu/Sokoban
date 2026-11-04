"""
macro_push.py — Macro Push + LACAM-lite cho Sokoban Solver

╔══════════════════════════════════════════════════════════════════════╗
║  MACRO PUSH                                                          ║
║  Thay vì duyệt R×4 bước đi robot (đa phần là đi bộ vô ích),          ║
║  chỉ xét B×4 lần ĐẨY THÙNG thực sự.                                  ║
║  Branching factor giảm từ hàng chục → vài đơn vị.                    ║
║                                                                      ║
║  LACAM-lite (Lazy Constraints Addition for Multi-Agent)              ║
║  Khi robot bị chặn bởi robot khác trên đường đến vị trí đẩy:         ║
║  - Ước lượng chi phí "clearing" (dọn robot chắn đường)               ║
║  - Trong reconstruction: thực sự di chuyển robot chắn sang ô khác    ║
║                                                                      ║
║  RECONSTRUCTION                                                      ║
║  Sau khi tìm được chuỗi push tối ưu, dựng lại đường đi               ║
║  step-by-step (gval +1 mỗi bước) để validator chấp nhận.             ║
╚══════════════════════════════════════════════════════════════════════╝
"""

from collections import deque
import heapq

from sokoban import SokobanState, sokoban_goal_state
import mapper

# ═════════════════════════════════════════════════════════════════════
#  CONSTANTS
# ═════════════════════════════════════════════════════════════════════

_DIRS = ((0, -1), (0, 1), (-1, 0), (1, 0))
_DIR_NAME = {(0, -1): "up", (0, 1): "down", (-1, 0): "left", (1, 0): "right"}


# ═════════════════════════════════════════════════════════════════════
#  BFS UTILITIES
# ═════════════════════════════════════════════════════════════════════

def _flood(start, blocked, w, h):
    """
    BFS flood fill từ start, trả về dict {position: distance}.
    Dùng để pre-compute toàn bộ ô robot có thể đi tới (O(W×H) mỗi lần).
    Sau đó tra cứu khoảng cách đến bất kỳ ô nào chỉ tốn O(1).
    """
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


def _bfs_path(start, target, blocked, w, h):
    """
    BFS tìm đường ngắn nhất [start, ..., target] hoặc None.
    Dùng trong reconstruction (cần biết từng bước đi cụ thể).
    """
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
#  MACRO PUSH — SUCCESSOR GENERATION
# ═════════════════════════════════════════════════════════════════════

def macro_successors(state, map_info):
    """
    Sinh tất cả trạng thái kế bằng Macro Push.

    Thay vì duyệt R×4 bước đi robot:
        Chỉ xét B×4 lần ĐẨY THÙNG (B thùng × 4 hướng).
        Mỗi successor = 1 robot đi đến vị trí đẩy + đẩy thùng 1 ô.
        Chi phí thực = bước đi robot (BFS) + 1 bước đẩy.

    Hai cấp độ tìm kiếm robot:
        Level 1 (Strict): BFS với robot khác là vật cản → chi phí chính xác
        Level 2 (LACAM):  BFS bỏ qua robot khác → ước lượng chi phí clearing

    Returns: list of (new_SokobanState, actual_cost)
    """
    dead_sq = map_info.dead_squares
    w, h = state.width, state.height
    obs = state.obstacles
    boxes = state.boxes
    robots = state.robots
    nr = len(robots)
    storage = state.storage
    robot_set = frozenset(robots)

    # ── Pre-compute reachability cho mỗi robot (1 BFS flood fill / robot) ──
    # Level 1: Strict — tường + thùng + robot khác đều là vật cản
    reach_strict = []
    for r in range(nr):
        other_r = frozenset(robots[j] for j in range(nr) if j != r)
        reach_strict.append(_flood(robots[r], obs | boxes | other_r, w, h))

    # Level 2: Relaxed — chỉ tường + thùng là vật cản (bỏ qua robot khác)
    # Dùng cho LACAM fallback
    reach_relaxed = []
    blocked_base = obs | boxes
    for r in range(nr):
        reach_relaxed.append(_flood(robots[r], blocked_base, w, h))

    strict_moves = []
    relaxed_moves = []

    for box in boxes:
        bx, by = box
        for dx, dy in _DIRS:
            # ── Vị trí mới của thùng sau khi đẩy ──
            nb = (bx + dx, by + dy)
            if not (0 <= nb[0] < w and 0 <= nb[1] < h):
                continue
            if nb in obs or nb in boxes:
                continue
            # Dead square pruning: thùng rơi vào ô chết → bỏ qua
            if nb in dead_sq:
                continue

            # ── Vị trí robot phải đứng để đẩy (đối diện hướng đẩy) ──
            pp = (bx - dx, by - dy)
            if not (0 <= pp[0] < w and 0 <= pp[1] < h):
                continue
            if pp in obs or pp in boxes:
                continue

            # ── SỬA THIẾU SÓT 5: Duyệt TẤT CẢ robot có thể đến được pp (Level 1) ──
            found_strict = False
            for r in range(nr):
                # Thùng không thể đẩy vào vị trí robot KHÁC đang đứng
                if nb in robot_set and nb != robots[r]:
                    continue
                if pp in reach_strict[r]:
                    found_strict = True
                    walk_cost = reach_strict[r][pp]
                    total_cost = walk_cost + 1

                    new_robots = list(robots)
                    new_robots[r] = box  # Robot r chiếm vị trí cũ của thùng sau khi đẩy
                    new_boxes = frozenset((boxes - {box}) | {nb})

                    ns = SokobanState(
                        f"{r} push_{_DIR_NAME[(dx, dy)]}",
                        state.gval + total_cost,
                        state,  # parent = macro state trước đó (để truy vết)
                        w, h, tuple(new_robots), new_boxes,
                        storage, obs
                    )
                    strict_moves.append((ns, total_cost))

            # ── SỬA THIẾU SÓT 4: Chỉ xét Level 2 (LACAM) khi hướng đẩy này chưa có robot nào đi được Level 1 ──
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
                        w, h, tuple(new_robots), new_boxes,
                        storage, obs
                    )
                    relaxed_moves.append((ns, total_cost))

    # ── SỬA THIẾU SÓT 4 (Tiếp): Ưu tiên tuyệt đối strict_moves để ngăn State Desynchronization ──
    # Chỉ khi toàn bộ các hướng đẩy của các thùng đều bị robot khác cản thì mới dùng relaxed_moves
    if strict_moves:
        return strict_moves
    return relaxed_moves


# ═════════════════════════════════════════════════════════════════════
#  RECONSTRUCTION: Macro Chain → Step-by-Step Path
# ═════════════════════════════════════════════════════════════════════

def _step_robot(current, robot_idx, new_pos, w, h):
    """Tạo SokobanState mới khi robot di chuyển 1 ô (không đẩy thùng)."""
    old_pos = current.robots[robot_idx]
    dx = new_pos[0] - old_pos[0]
    dy = new_pos[1] - old_pos[1]

    new_robots = list(current.robots)
    new_robots[robot_idx] = new_pos

    return SokobanState(
        f"{robot_idx} {_DIR_NAME[(dx, dy)]}",
        current.gval + 1,
        current,
        w, h, tuple(new_robots), current.boxes,
        current.storage, current.obstacles
    )


def _walk_robot_to(current, robot_idx, target, w, h):
    """
    Di chuyển robot step-by-step đến target.
    Nếu bị chặn bởi robot khác → LACAM: dọn robot chắn đường trước.

    Returns: (final_state, success: bool)
    """
    robots = current.robots
    nr = len(robots)

    if robots[robot_idx] == target:
        return current, True

    obs = current.obstacles
    boxes = current.boxes
    other_r = frozenset(robots[j] for j in range(nr) if j != robot_idx)

    # ── Thử đường đi trực tiếp (Level 1) ──
    path = _bfs_path(robots[robot_idx], target, obs | boxes | other_r, w, h)

    if path is not None:
        cur = current
        for i in range(1, len(path)):
            cur = _step_robot(cur, robot_idx, path[i], w, h)
        return cur, True

    # ── LACAM: Dọn robot chắn đường ──
    ideal = _bfs_path(robots[robot_idx], target, obs | boxes, w, h)
    if ideal is None:
        return current, False  # Không thể đến được (bị tường/thùng chặn hoàn toàn)

    # Tìm robot nào đang chắn trên đường đi lý tưởng
    ideal_cells = set(ideal[1:])  # Bỏ vị trí xuất phát (robot đang đứng)
    blockers = [(j, robots[j]) for j in range(nr)
                if j != robot_idx and robots[j] in ideal_cells]

    cur = current

    for b_idx, _b_pos in blockers:
        b_pos = cur.robots[b_idx]  # Vị trí hiện tại (có thể đã thay đổi)
        if b_pos not in ideal_cells:
            continue  # Robot này đã di chuyển ra khỏi đường rồi

        # Tìm ô trống gần nhất KHÔNG nằm trên ideal path
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
                    q2.append(np2)  # Ô trống nhưng trên ideal path → tiếp tục tìm
            if free_target:
                break

        if free_target is None:
            continue  # Không tìm được chỗ trống → bỏ qua robot này

        # Di chuyển robot chặn đến ô trống
        other_b = frozenset(cur.robots[j] for j in range(nr) if j != b_idx)
        clear_path = _bfs_path(cur.robots[b_idx], free_target,
                               obs | cur.boxes | other_b, w, h)
        if clear_path is None:
            continue

        for i in range(1, len(clear_path)):
            cur = _step_robot(cur, b_idx, clear_path[i], w, h)

    # ── Thử lại đường đi sau khi dọn robot ──
    other_r2 = frozenset(cur.robots[j] for j in range(nr) if j != robot_idx)
    path2 = _bfs_path(cur.robots[robot_idx], target, obs | cur.boxes | other_r2, w, h)

    if path2 is None:
        return current, False  # Vẫn không đi được → thất bại

    for i in range(1, len(path2)):
        cur = _step_robot(cur, robot_idx, path2[i], w, h)

    return cur, True


def reconstruct(initial_state, macro_goal):
    """
    Chuyển chuỗi macro push → đường đi step-by-step đầy đủ.

    Validator yêu cầu:
      1. Mỗi cặp state liên tiếp có gval chênh đúng 1
      2. Mỗi transition là một successor hợp lệ của framework
      3. State cuối cùng là goal

    Hàm này dựng lại TOÀN BỘ intermediate states (robot đi bộ + đẩy thùng).

    Returns: goal SokobanState với parent chain đầy đủ, hoặc False
    """
    # Truy vết macro chain: goal → ... → initial
    chain = []
    cur = macro_goal
    while cur is not None:
        chain.append(cur)
        cur = cur.parent
    chain.reverse()  # initial → ... → goal

    if len(chain) <= 1:
        return initial_state

    w, h = initial_state.width, initial_state.height

    # Khởi tạo state đầu tiên (gval=0, parent=None) 
    current = SokobanState(
        "START", 0, None,
        w, h, chain[0].robots, chain[0].boxes,
        chain[0].storage, chain[0].obstacles
    )

    for k in range(1, len(chain)):
        prev_m = chain[k - 1]
        next_m = chain[k]

        # ── Xác định push nào đã xảy ra ──
        removed = prev_m.boxes - next_m.boxes  # Thùng biến mất (vị trí cũ)
        added = next_m.boxes - prev_m.boxes    # Thùng xuất hiện (vị trí mới)
        if not removed or not added:
            continue

        old_box = next(iter(removed))
        new_box = next(iter(added))
        dx = new_box[0] - old_box[0]
        dy = new_box[1] - old_box[1]
        push_pos = (old_box[0] - dx, old_box[1] - dy)

        # ── Xác định robot nào đã đẩy ──
        push_r = None
        for r in range(len(next_m.robots)):
            if next_m.robots[r] == old_box:
                # Robot này kết thúc tại vị trí thùng cũ → nó là robot đẩy
                push_r = r
                break
        if push_r is None:
            return False

        # ── Bước 1: Di chuyển robot đến push_pos (có LACAM nếu cần) ──
        current, ok = _walk_robot_to(current, push_r, push_pos, w, h)
        if not ok:
            return False

        # ── Bước 2: Thực hiện push (1 bước) ──
        new_robots = list(current.robots)
        new_robots[push_r] = old_box  # Robot di chuyển vào vị trí thùng cũ
        new_boxes = frozenset((current.boxes - {old_box}) | {new_box})

        current = SokobanState(
            f"{push_r} {_DIR_NAME[(dx, dy)]}",
            current.gval + 1,
            current,
            w, h, tuple(new_robots), new_boxes,
            current.storage, current.obstacles
        )

    return current


# ═════════════════════════════════════════════════════════════════════
#  MACRO A* SEARCH
# ═════════════════════════════════════════════════════════════════════

class _Node:
    """Wrapper cho PriorityQueue với tie-breaking: f nhỏ → h nhỏ."""
    __slots__ = ('state', 'f', 'h')

    def __init__(self, s, f, h):
        self.state = s
        self.f = f
        self.h = h

    def __lt__(self, other):
        if self.f != other.f:
            return self.f < other.f
        return self.h < other.h  # Tie-break: ưu tiên h nhỏ (gần đích hơn)


def default_heuristic(state):
    """
    SỬA THIẾU SÓT 2: Heuristic kết hợp dẫn đường mạnh mẽ:
    1. AdvancedHeuristic: Ghép cặp 1-1 tối ưu Thùng -> Đích (Box assignment).
    2. RobotHeuristic (guided): Điều phối robot tiếp cận các thùng chưa vào đích.
    """
    try:
        from AdvancedHeuristic import advanced_heuristic
        import RobotHeuristic as RH
        hb = advanced_heuristic(state)
        if hb == float('inf'):
            return float('inf')
        hr = RH.robot_to_box_heuristic(state, mode="guided")
        return hb + hr
    except Exception:
        from RobotHeuristic import combined_heuristic
        return combined_heuristic(state, mode="guided")


def macro_astar(initial_state, w=2.0, timebound=120, heuristic_fn=None):
    """
    Weighted A* sử dụng Macro Push làm successor generator.

    Mỗi successor = 1 lần đẩy thùng (thay vì 1 bước đi robot).
    Sau khi tìm được chuỗi push dẫn đến goal, reconstruct lại
    đường đi step-by-step đầy đủ cho validator.

    Args:
        initial_state: SokobanState ban đầu
        w: trọng số heuristic (w=1.0 = A* chuẩn, w>1 = Weighted A*)
        timebound: giới hạn thời gian (giây) — hiện chưa enforce
        heuristic_fn: hàm heuristic(state) → float

    Returns:
        Goal SokobanState với parent chain step-by-step, hoặc False
    """
    if heuristic_fn is None:
        heuristic_fn = default_heuristic

    mi = mapper.get_map_info(initial_state)

    if mapper.is_deadlock(initial_state, mi):
        return False

    h0 = heuristic_fn(initial_state)
    if h0 == float('inf'):
        return False

    pq = [_Node(initial_state, initial_state.gval + w * h0, h0)]
    explored = {initial_state.hashable_state(): initial_state.gval}

    while pq:
        node = heapq.heappop(pq)
        st = node.state
        key = st.hashable_state()

        # Skip nếu đã tìm thấy đường tốt hơn đến state này
        if st.gval > explored.get(key, float('inf')):
            continue

        if sokoban_goal_state(st):
            # Tìm thấy goal! Reconstruct đường đi step-by-step.
            return reconstruct(initial_state, st)

        for ns, _ in macro_successors(st, mi):
            nk = ns.hashable_state()
            ng = ns.gval

            if nk in explored and ng >= explored[nk]:
                continue

            # Deadlock pruning (2x2 freeze, corridor, blocked goal, ...)
            if mapper.is_deadlock(ns, mi):
                continue

            hv = heuristic_fn(ns)
            if hv == float('inf'):
                continue

            explored[nk] = ng
            heapq.heappush(pq, _Node(ns, ng + w * hv, hv))

    return False
