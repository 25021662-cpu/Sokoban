"""
tune_w.py - Thực nghiệm đánh giá hệ số w trong khoảng từ 1.7 đến 3.0
Đo lường chi tiết:
- Số test PASS / 20
- Tổng số bước đi (Total Steps / Solution Quality)
- Tổng thời gian chạy (Total Time)
- Thời gian trung bình / test
- Test tốn thời gian nhất (Max Time per test)
"""

import time
import sys
from sokoban import PROBLEMS
from validation import validate_solution
import solution_submit

w_candidates = [2.1, 2.2, 2.3, 2.4, 2.5]
results = []

print("=" * 86)
print(f" BẮT ĐẦU THỰC NGHIỆM ĐÁNH GIÁ w TRONG KHOẢNG [2.0, 2.5]")
print(f" Số lượng map thử nghiệm: {len(PROBLEMS)} problems")
print("=" * 86)

for w in w_candidates:
    print(f"\n>> Đang chạy thực nghiệm với w = {w:.1f} ...", flush=True)
    total_time = 0.0
    total_steps = 0
    passed = 0
    max_test_time = 0.0
    max_test_idx = -1
    step_details = []

    for i, prob in enumerate(PROBLEMS):
        t0 = time.time()
        res = solution_submit.solve(prob, timebound=60, w=w)
        elapsed = time.time() - t0
        total_time += elapsed

        if elapsed > max_test_time:
            max_test_time = elapsed
            max_test_idx = i

        if res and res is not False:
            valid, _ = validate_solution(prob, res)
            if valid:
                passed += 1
                total_steps += res.gval
                step_details.append(res.gval)
            else:
                step_details.append(None)
        else:
            step_details.append(None)

    avg_steps = (total_steps / passed) if passed > 0 else 0
    avg_time = (total_time / len(PROBLEMS))

    summary = {
        "w": w,
        "passed": passed,
        "total_steps": total_steps,
        "avg_steps": round(avg_steps, 2),
        "total_time": round(total_time, 4),
        "avg_time": round(avg_time, 4),
        "max_test_time": round(max_test_time, 4),
        "max_test_idx": max_test_idx,
        "step_details": step_details
    }
    results.append(summary)

    print(f"   => w={w:.1f}: Passed={passed}/20 | Tổng bước={total_steps} (TB={avg_steps:.1f}) | Tổng thời gian={total_time:.3f}s (Max Test {max_test_idx}: {max_test_time:.3f}s)")

print("\n" + "=" * 86)
print(" BẢNG TỔNG KẾT SO SÁNH CÁC GIÁ TRỊ w:")
print("=" * 86)
print(f" {'w':<6} | {'Passed':<8} | {'Tổng bước':<10} | {'TB bước':<10} | {'Tổng time':<12} | {'TB time':<10} | {'Max test time':<14}")
print("-" * 86)
for r in results:
    print(f" {r['w']:<6.1f} | {r['passed']:<2}/20     | {r['total_steps']:<10} | {r['avg_steps']:<10.2f} | {r['total_time']:<8.3f}s   | {r['avg_time']:<6.3f}s   | Test {r['max_test_idx']}: {r['max_test_time']:.3f}s")
print("=" * 86)
