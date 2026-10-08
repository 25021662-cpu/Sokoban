import time
import os
import pandas as pd
from parser import parse_level
from solution import solve

def run_benchmark(level_folder, output_csv="benchmark_results.csv", timebound=120):
    """
    Quét qua tất cả file .lvl, chạy thuật toán và xuất CSV.
    """
    print(f"Bắt đầu quét thư mục: {level_folder}")
    print(f"Kết quả sẽ được lưu vào file: {output_csv}")
    print("-" * 50)
    
    files = [f for f in os.listdir(level_folder) if f.endswith('.lvl')][:25]
    results = [] # List chứa các Dictionary dữ liệu
    
    for filename in files:
        level_path = os.path.join(level_folder, filename)
        
        try:
            initial_state = parse_level(level_path)
        except Exception as e:
            print(f"[{filename:<30}] ⚠️ PERR")
            results.append({
                "Level Name": filename,
                "Status": "PERR",
                "Length (Steps)": None,
                "Time (s)": None
            })
            continue
        
        start_time = time.time()
        result_state = solve(initial_state, timebound)
        elapsed_time = time.time() - start_time
        
        if elapsed_time >= timebound:
            print(f"[{filename:<30}] ⏳ TIMEOUT")
            results.append({
                "Level Name": filename,
                "Status": "TIMEOUT",
                "Length (Steps)": None,
                "Time (s)": round(elapsed_time, 4)
            })
        elif result_state:
            steps = result_state.gval
            print(f"[{filename:<30}] ✅ PASS (Steps: {steps:<4} | Time: {elapsed_time:.4f}s)")
            results.append({
                "Level Name": filename,
                "Status": "PASS",
                "Length (Steps)": steps,
                "Time (s)": round(elapsed_time, 4)
            })
        else:
            print(f"[{filename:<30}] ❌ FAIL (Elapsed Time: {elapsed_time:.4f}s)")
            results.append({
                "Level Name": filename,
                "Status": "FAIL",
                "Length (Steps)": None,
                "Time (s)": round(elapsed_time, 4)
            })

    df = pd.DataFrame(results)
    
    # Xuất ra file CSV
    df.to_csv(output_csv, index=False, encoding='utf-8-sig')

    # In Tổng kết bằng sức mạnh thống kê của Pandas
    print("-" * 50)
    print("🏆 BÁO CÁO TỔNG KẾT:")
    
    total_tests = len(df)
    status_counts = df['Status'].value_counts()
    passed_tests = status_counts.get('PASS', 0)
    
    print(f"   - Số test Passed: {passed_tests}/{total_tests}")
    
    if passed_tests > 0:
        # Lọc ra các test Pass và tính Mean
        success_df = df[df['Status'] == 'PASS']
        avg_steps = success_df['Length (Steps)'].mean()
        avg_time = success_df['Time (s)'].mean()
        print(f"   - Trung bình số bước / test: {avg_steps:.2f} steps")
        print(f"   - Trung bình thời gian / test: {avg_time:.4f} giây")
        
    print(f"   => Đã xuất thành công file '{output_csv}'!")

if __name__ == "__main__":
    folder_path = "levels/competition_levels"
    output_file = "benchmark/benchmark_results.csv"
    
    if os.path.exists(folder_path):
        # Chạy benchmark với timebound=10s để quét nhanh
        run_benchmark(folder_path, output_csv=output_file)
    else:
        print(f"Không tìm thấy thư mục map tại: {folder_path}")
