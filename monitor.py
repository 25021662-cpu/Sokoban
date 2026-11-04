"""
monitor.py - Công cụ đo lường và theo dõi tiến trình tìm kiếm theo chu kỳ thời gian (t giây).

Tính năng:
- Hỗ trợ đo lường từng thuật toán: BFS, DFS, A*, A*W (Weighted A*).
- Cứ sau mỗi khoảng thời gian t giây (mặc định 1.0s), in ra màn hình:
  + Thời gian trôi qua (Elapsed time)
  + Số trạng thái đã mở rộng (Explored / Pop count)
  + Số trạng thái đang chờ trong hàng đợi (Frontier / Queue size)
  + Tổng số trạng thái từng nhìn thấy (Seen states)
  + Tốc độ duyệt trạng thái (states/giây)
  + Chi phí trạng thái hiện tại (g, h, f)
- Báo cáo kết quả: PASS (kèm gval, đường đi) / TIMEOUT / FAIL.

Cách dùng qua dòng lệnh:
  python monitor.py --algo astarw --map 17 -t 1.0
  python monitor.py --algo bfs --map 5 -t 0.5
  python monitor.py --algo dfs --map 1
  python monitor.py --algo astarw --lvl levels/default_levels/SAsimple1.lvl
"""

import argparse
import heapq
import os
import sys
import time
from collections import deque
from typing import Any, Callable, Dict, List, Optional, Tuple

import mapper
from sokoban import Direction, PROBLEMS, SokobanState, sokoban_goal_state

try:
    import AdvancedHeuristic as AH
except ImportError:
    AH = None

try:
    import RobotHeuristic as RH
except ImportError:
    RH = None

try:
    import BasicBoxHeuristic as BH
except ImportError:
    BH = None


# Node wrapper có tie-breaking tối ưu cho PriorityQueue
class PriorityNode:
    def __init__(self, state: Any, f_value: float, h_value: float):
        self.state = state
        self.f_value = f_value
        self.h_value = h_value

    def __lt__(self, other: "PriorityNode") -> bool:
        if self.f_value == other.f_value:
            return self.h_value < other.h_value
        return self.f_value < other.f_value


def get_heuristic_function(name: str) -> Callable[[Any], float]:
    """Lấy hàm heuristic theo tên lựa chọn."""
    if name == "advanced":
        if AH and hasattr(AH, "advanced_heuristic"):
            if RH and hasattr(RH, "robot_to_box_heuristic"):
                def h_adv(s):
                    hb = AH.advanced_heuristic(s)
                    if hb == float("inf"):
                        return float("inf")
                    return hb + RH.robot_to_box_heuristic(s, mode="guided")
                return h_adv
            return AH.advanced_heuristic
        raise ValueError("Không tìm thấy AdvancedHeuristic.py!")

    elif name == "combined":
        if RH and hasattr(RH, "combined_heuristic"):
            return lambda s: RH.combined_heuristic(s, mode="guided")
        raise ValueError("Không tìm thấy RobotHeuristic.py!")

    elif name == "box_only":
        if RH and hasattr(RH, "box_to_storage_heuristic"):
            return RH.box_to_storage_heuristic
        raise ValueError("Không tìm thấy box_to_storage_heuristic!")

    elif name == "zero":
        return lambda s: 0.0

    else:
        # Mặc định: Dùng advanced nếu có, ngược lại combined
        if AH and hasattr(AH, "advanced_heuristic"):
            if RH and hasattr(RH, "robot_to_box_heuristic"):
                def h_default(s):
                    hb = AH.advanced_heuristic(s)
                    if hb == float("inf"):
                        return float("inf")
                    return hb + RH.robot_to_box_heuristic(s, mode="guided")
                return h_default
            return AH.advanced_heuristic
        if RH and hasattr(RH, "combined_heuristic"):
            return lambda s: RH.combined_heuristic(s, mode="guided")
        return lambda s: 0.0


def monitor_search(
    algo: str,
    initial_state: SokobanState,
    timebound: float = 120.0,
    interval: float = 1.0,
    w: float = 2.0,
    heuristic_name: str = "default",
    prune_deadlocks: bool = True,
) -> Optional[SokobanState]:
    """
    Hàm theo dõi tiến trình tìm kiếm tổng quát.
    In trạng thái định kỳ mỗi khoảng thời gian `interval` (giây).
    """
    algo = algo.lower().strip()
    h_fn = get_heuristic_function(heuristic_name)
    map_info = mapper.get_map_info(initial_state)

    print(f"\n{'=' * 92}")
    print(f" THUẬT TOÁN: {algo.upper():<8} | BẢN ĐỒ: {initial_state.width}x{initial_state.height} | THỜI GIAN TỐI ĐA: {timebound}s | CHU KỲ GHI NHẬN: {interval}s")
    if algo in ("astar", "astarw"):
        print(f" CẤU HÌNH A*: Trọng số w={w} | Heuristic: '{heuristic_name}' | Cắt tỉa Deadlock: {prune_deadlocks}")
    print(f"{'=' * 92}")
    print(f"{'Thời gian':^12} | {'Explored':^12} | {'Frontier':^12} | {'Seen (Tập mở)':^14} | {'Tốc độ':^14} | {'Chi phí hiện tại':^16}")
    print(f"{'-' * 92}")

    start_time = time.time()
    last_report_time = start_time
    pop_count = 0
    last_pop_count = 0

    # --------------------------------------------------------------------------
    # THUẬT TOÁN 1: BFS (Tìm kiếm theo chiều rộng)
    # --------------------------------------------------------------------------
    if algo == "bfs":
        queue = deque([initial_state])
        explored = {initial_state.hashable_state()}

        while queue:
            now = time.time()
            elapsed = now - start_time
            if elapsed > timebound:
                _print_timeout(elapsed, timebound)
                return None

            if now - last_report_time >= interval:
                dt = now - last_report_time
                speed = (pop_count - last_pop_count) / dt if dt > 0 else 0
                last_report_time = now
                last_pop_count = pop_count
                curr_sample = queue[0]
                print(f"{elapsed:9.2f}s   | {pop_count:12,d} | {len(queue):12,d} | {len(explored):14,d} | {speed:8.0f} st/s | g={curr_sample.gval:<3}")

            current_state = queue.popleft()
            pop_count += 1

            if sokoban_goal_state(current_state):
                _print_success(start_time, pop_count, len(queue), current_state)
                return current_state

            for next_state in current_state.successors():
                key = next_state.hashable_state()
                if key not in explored:
                    if prune_deadlocks and mapper.is_deadlock(next_state, map_info):
                        continue
                    explored.add(key)
                    queue.append(next_state)

        _print_exhausted(start_time, pop_count)
        return False

    # --------------------------------------------------------------------------
    # THUẬT TOÁN 2: DFS (Tìm kiếm theo chiều sâu)
    # --------------------------------------------------------------------------
    elif algo == "dfs":
        stack = [initial_state]
        explored = {initial_state.hashable_state()}

        while stack:
            now = time.time()
            elapsed = now - start_time
            if elapsed > timebound:
                _print_timeout(elapsed, timebound)
                return None

            if now - last_report_time >= interval:
                dt = now - last_report_time
                speed = (pop_count - last_pop_count) / dt if dt > 0 else 0
                last_report_time = now
                last_pop_count = pop_count
                curr_sample = stack[-1]
                print(f"{elapsed:9.2f}s   | {pop_count:12,d} | {len(stack):12,d} | {len(explored):14,d} | {speed:8.0f} st/s | g={curr_sample.gval:<3}")

            current_state = stack.pop()
            pop_count += 1

            if sokoban_goal_state(current_state):
                _print_success(start_time, pop_count, len(stack), current_state)
                return current_state

            for next_state in current_state.successors():
                key = next_state.hashable_state()
                if key not in explored:
                    if prune_deadlocks and mapper.is_deadlock(next_state, map_info):
                        continue
                    explored.add(key)
                    stack.append(next_state)

        _print_exhausted(start_time, pop_count)
        return False

    # --------------------------------------------------------------------------
    # THUẬT TOÁN 3 & 4: A* / Weighted A* (Tìm kiếm Heuristic)
    # --------------------------------------------------------------------------
    elif algo in ("astar", "astarw"):
        weight = 1.0 if algo == "astar" else w
        pq: List[PriorityNode] = []
        h_init = h_fn(initial_state)

        if h_init == float("inf"):
            print("❌ Trạng thái ban đầu có chi phí heuristic vô cùng (inf)!")
            return False

        f_init = initial_state.gval + weight * h_init
        heapq.heappush(pq, PriorityNode(initial_state, f_init, h_init))
        explored = {initial_state.hashable_state(): initial_state.gval}

        while pq:
            now = time.time()
            elapsed = now - start_time
            if elapsed > timebound:
                _print_timeout(elapsed, timebound)
                return None

            if now - last_report_time >= interval:
                dt = now - last_report_time
                speed = (pop_count - last_pop_count) / dt if dt > 0 else 0
                last_report_time = now
                last_pop_count = pop_count
                top_item = pq[0]
                print(f"{elapsed:9.2f}s   | {pop_count:12,d} | {len(pq):12,d} | {len(explored):14,d} | {speed:8.0f} st/s | g={top_item.state.gval:<3} h={top_item.h_value:<4.1f} f={top_item.f_value:<5.1f}")

            curr_node = heapq.heappop(pq)
            current_state = curr_node.state
            pop_count += 1

            if current_state.gval > explored.get(current_state.hashable_state(), float("inf")):
                continue

            if sokoban_goal_state(current_state):
                _print_success(start_time, pop_count, len(pq), current_state)
                return current_state

            for next_state in current_state.successors():
                next_key = next_state.hashable_state()
                new_g = next_state.gval

                if next_key not in explored or new_g < explored[next_key]:
                    if prune_deadlocks and mapper.is_deadlock(next_state, map_info):
                        continue

                    h_val = h_fn(next_state)
                    if h_val == float("inf"):
                        continue

                    explored[next_key] = new_g
                    f_val = new_g + weight * h_val
                    heapq.heappush(pq, PriorityNode(next_state, f_val, h_val))

        _print_exhausted(start_time, pop_count)
        return False

    else:
        print(f"❌ Thuật toán không hợp lệ: '{algo}'. Vui lòng chọn trong: [bfs, dfs, astar, astarw]")
        return False


def _print_success(start_time: float, pop_count: int, frontier_len: int, goal_state: SokobanState):
    total_time = time.time() - start_time
    print(f"{'-' * 92}")
    print(f"✅ THÀNH CÔNG! Thời gian: {total_time:.4f}s | Số bước (gval): {goal_state.gval}")
    print(f"   - Tổng trạng thái đã mở rộng (Explored): {pop_count:,d}")
    print(f"   - Số trạng thái còn trong hàng đợi: {frontier_len:,d}")
    print(f"{'=' * 92}\n")


def _print_timeout(elapsed: float, timebound: float):
    print(f"{'-' * 92}")
    print(f"⏳ TIMEOUT! Thuật toán đã vượt quá thời gian tối đa ({elapsed:.2f}s >= {timebound}s).")
    print(f"{'=' * 92}\n")


def _print_exhausted(start_time: float, pop_count: int):
    total_time = time.time() - start_time
    print(f"{'-' * 92}")
    print(f"❌ THẤT BẠI: Hàng đợi rỗng (Frontier empty) sau {total_time:.3f}s! Mở được {pop_count:,d} trạng thái.")
    print(f"{'=' * 92}\n")


# ==============================================================================
# HÀM CHẠY TRỰC TIẾP QUA CLI (COMMAND LINE INTERFACE)
# ==============================================================================
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Công cụ theo dõi và đo lường thuật toán giải Sokoban.")
    parser.add_argument("--algo", type=str, default="astarw", choices=["bfs", "dfs", "astar", "astarw"], help="Thuật toán cần test.")
    parser.add_argument("--map", type=int, default=17, help="Chỉ số map trong mảng PROBLEMS (0 - 19).")
    parser.add_argument("--lvl", type=str, default=None, help="Đường dẫn đến file .lvl (tùy chọn).")
    parser.add_argument("-t", "--interval", type=float, default=1.0, help="Khoảng thời gian (giây) in cập nhật 1 lần.")
    parser.add_argument("--timebound", type=float, default=60.0, help="Thời gian tối đa (giây).")
    parser.add_argument("-w", type=float, default=2.0, help="Hệ số trọng số w của Weighted A*.")
    parser.add_argument("--heuristic", type=str, default="advanced", choices=["default", "advanced", "combined", "box_only", "zero"], help="Hàm heuristic cần dùng.")
    parser.add_argument("--no-prune", action="store_true", help="Tắt tính năng cắt tỉa deadlock.")

    args = parser.parse_args()

    # Nạp map cần chạy
    if args.lvl and os.path.exists(args.lvl):
        try:
            from parser import parse_level
            problem_state = parse_level(args.lvl)
            print(f"Đã nạp bản đồ từ file: {args.lvl}")
        except Exception as e:
            print(f"Lỗi khi đọc file {args.lvl}: {e}")
            sys.exit(1)
    else:
        if 0 <= args.map < len(PROBLEMS):
            problem_state = PROBLEMS[args.map]
            print(f"Đã nạp Problem [{args.map}] trong PROBLEMS (size: {problem_state.width}x{problem_state.height}).")
        else:
            print(f"Chỉ số map không hợp lệ: {args.map} (chỉ có từ 0 đến {len(PROBLEMS)-1}).")
            sys.exit(1)

    # Hiển thị bản đồ
    problem_state.print_state()

    # Thực thi đo lường
    monitor_search(
        algo=args.algo,
        initial_state=problem_state,
        timebound=args.timebound,
        interval=args.interval,
        w=args.w,
        heuristic_name=args.heuristic,
        prune_deadlocks=not args.no_prune,
    )
