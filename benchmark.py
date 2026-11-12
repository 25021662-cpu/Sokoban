import time
import os
import pandas as pd
from parser import parse_level
from solution_submit import solve
from validation import validate_solution

def run_benchmark(level_folder, output_csv, w, show_map=False, timebound=5, validate=True):
    """
    Quét qua tất cả file .lvl, chạy thuật toán và xuất CSV.
    """
    print(f"Bắt đầu quét thư mục: {level_folder}")
    print(w, timebound)
    if output_csv:
        print(f"Kết quả sẽ được lưu vào file: {output_csv}")
    # print("-" * 72)
    
    files = sorted([f for f in os.listdir(level_folder) if f.endswith('.lvl')])
    results = [] # List chứa các Dictionary dữ liệu
    
    for filename in files:
        level_path = os.path.join(level_folder, filename)
        
        try:
            initial_state = parse_level(level_path)
        except Exception as e:
            print(f"[{filename:<30}] ⚠️⚠️ PERR")
            results.append({
                "Level Name": filename,
                "Status": "PERR",
                "Length (Steps)": None,
                "Time (s)": None
            })
            continue
        
        # print("-" * 72)
        if show_map:
            initial_state.print_state()
        print(f"[{filename:<30}]", end=" ", flush=True)       
        
        start_time = time.time()
        result_state = solve(initial_state, timebound, w)
        elapsed_time = time.time() - start_time
        
        if elapsed_time >= timebound:
            print("⏳ TIMEOUT")
            results.append({
                "Level Name": filename,
                "Status": "TIMEOUT",
                "Length (Steps)": None,
                "Time (s)": round(elapsed_time, 4)
            })
        elif result_state:
            is_valid = True
            val_msg = "Valid solution"
            if validate:
                try:
                    is_valid, val_msg = validate_solution(initial_state, result_state)
                except Exception as e:
                    is_valid, val_msg = False, f"Validation error: {e}"

            if is_valid:
                steps = result_state.gval
                print(f"✅ PASS (Steps: {steps:<4} | Time: {elapsed_time:.4f}s)")
                results.append({
                    "Level Name": filename,
                    "Status": "PASS",
                    "Length (Steps)": steps,
                    "Time (s)": round(elapsed_time, 4)
                })
            else:
                steps = getattr(result_state, 'gval', None)
                print(f"❌ INVALID ({val_msg} | Steps: {steps} | Time: {elapsed_time:.4f}s)")
                results.append({
                    "Level Name": filename,
                    "Status": "INVALID",
                    "Length (Steps)": None,
                    "Time (s)": round(elapsed_time, 4)
                })
        else:
            print(f"❌ FAIL (Elapsed Time: {elapsed_time:.4f}s)")
            results.append({
                "Level Name": filename,
                "Status": "FAIL",
                "Length (Steps)": None,
                "Time (s)": round(elapsed_time, 4)
            })

    df = pd.DataFrame(results)
    
    # Xuất ra file CSV
    if output_csv:
        out_dir = os.path.dirname(output_csv)
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)
        df.to_csv(output_csv, index=False, encoding='utf-8')
        print(f"   => Đã xuất thành công file '{output_csv}'!")

    print("-" * 72)
    print("🏆 BÁO CÁO TỔNG KẾT:")
    
    total_tests = len(df)
    status_counts = df['Status'].value_counts()
    passed_tests = status_counts.get('PASS', 0)
    invalid_tests = status_counts.get('INVALID', 0)
    failed_tests = status_counts.get('FAIL', 0)
    timeout_tests = status_counts.get('TIMEOUT', 0)
    
    print(f"   - Số test Passed: {passed_tests}/{total_tests}")
    if invalid_tests > 0:
        print(f"   - Số test Invalid: {invalid_tests}/{total_tests}")
    if failed_tests > 0:
        print(f"   - Số test Failed: {failed_tests}/{total_tests}")
    if timeout_tests > 0:
        print(f"   - Số test Timeout: {timeout_tests}/{total_tests}")
    
    if passed_tests > 0:
        # Lọc ra các test Pass và tính Mean
        success_df = df[df['Status'] == 'PASS']
        avg_steps = success_df['Length (Steps)'].mean()
        avg_time = success_df['Time (s)'].mean()
        print(f"   - Trung bình số bước / test: {avg_steps:.2f} steps")
        print(f"   - Trung bình thời gian / test: {avg_time:.4f} giây")

if __name__ == "__main__":
    w = 5
    folder_path = "levels/XSokoban"  # Thay đổi đường dẫn tới thư mục chứa các file .lvl
    output_file = None
    show_map = False

    if os.path.exists(folder_path):
        run_benchmark(folder_path, output_file, w, show_map)
    else:
        print(f"Không tìm thấy thư mục map tại: {folder_path}")
