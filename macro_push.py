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
import time

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

def _local_may_split(c, in_region):
    """Kiểm tra cục bộ (vòng 8 ô): bỏ ô c khỏi vùng có THỂ làm tách vùng không."""
    x, y = c
    n, e, s, wv = (x, y - 1), (x + 1, y), (x, y + 1), (x - 1, y)
    ne, se, sw, nw = (x + 1, y - 1), (x + 1, y + 1), (x - 1, y + 1), (x - 1, y - 1)
    ring = [(n, ne, e), (e, se, s), (s, sw, wv), (wv, nw, n)]
    free4 = [p for p in (n, e, s, wv) if in_region(p)]
    if len(free4) <= 1:
        return False
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
        opts.append((c, d))
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
    """Sinh các cấu hình vị trí robot rảnh sau 1 push của robot r."""
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


def macro_successors(state, map_info):
    """
    Sinh tất cả trạng thái kế tiếp bằng Macro Push (hỗ trợ Ray-casting & Tunnel Macro).
    """
    dead_sq = map_info.dead_squares
    w, h = state.width, state.height
    obs = state.obstacles
    boxes = state.boxes
    robots = state.robots
    nr = len(robots)
    storage = state.storage

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

            cur_b = nb1
            k = 1
            in_tunnel = map_info.is_tunnel(nb1, dx, dy) if hasattr(map_info, 'is_tunnel') else False

            while True:
                if cur_b in storage:
                    if (k, cur_b) not in pushes:
                        pushes.append((k, cur_b))

                next_b = (cur_b[0] + dx, cur_b[1] + dy)
                if not (0 <= next_b[0] < w and 0 <= next_b[1] < h):
                    break
                if next_b in obs or next_b in boxes:
                    break
                if next_b not in storage and next_b in dead_sq:
                    break

                if in_tunnel and hasattr(map_info, 'is_tunnel') and not map_info.is_tunnel(next_b, dx, dy):
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
    """
    Chuyển chuỗi macro push -> đường đi step-by-step đầy đủ (chuẩn validator).
    """
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

        # Di chuyển các robot rảnh tới vị trí đã chọn TRƯỚC khi push
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
#  MACRO A* SEARCH
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


def default_heuristic(state, map_info=None):
    """
    Heuristic kết hợp dẫn đường mạnh mẽ:
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
        try:
            from RobotHeuristic import combined_heuristic
            return combined_heuristic(state, mode="guided")
        except Exception:
            return 0


def macro_astar(initial_state, w=2.3, timebound=120, heuristic_fn=None, map_info=None):
    """
    Weighted A* sử dụng Macro Push làm successor generator.
    """
    start_time = time.time()
    if heuristic_fn is None:
        heuristic_fn = default_heuristic

    if map_info is None:
        map_info = mapper.get_map_info(initial_state)

    if mapper.is_deadlock(initial_state, map_info):
        return False

    effective_w = max(w, 2.8) if len(initial_state.robots) > 4 else w

    h0 = heuristic_fn(initial_state)
    if h0 == float('inf'):
        return False

    pq = [_Node(initial_state, initial_state.gval + effective_w * h0, h0)]
    explored = {initial_state.hashable_state(): initial_state.gval}

    step_counter = 0
    while pq:
        step_counter += 1
        if (step_counter & 127) == 0:
            if time.time() - start_time >= timebound:
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

            if mapper.is_deadlock(ns, map_info):
                continue

            hv = heuristic_fn(ns)
            if hv == float('inf'):
                continue

            explored[nk] = ng
            heapq.heappush(pq, _Node(ns, ng + effective_w * hv, hv))

    return False
