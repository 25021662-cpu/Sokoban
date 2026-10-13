# Tất cả thành viên cần hiểu rõ cách các API hoạt động, 1 State lưu những gì, mình đang làm gì, ... Cuối mỗi giai đoạn họp nhóm review lại toàn bộ code

## Giai đoạn 1: Xây dựng nền móng & Thuật toán cơ bản (Blind Search)

Mục tiêu: Hiểu rõ framework, dựng được luồng chạy cơ bản, vượt qua các test dễ nhất bằng thuật toán tìm kiếm mù, việc có 2-3 thuật toán mù trong giai đoạn đầu sẽ giúp team dễ dàng đối chiếu kết quả (cross-check) xem code sinh trạng thái có bị lỗi ở đâu không.

+ Nhi: (BFS Implementation) Viết thuật toán BFS (Tìm kiếm theo chiều rộng). Đảm bảo quản lý tốt tập hợp explored (các trạng thái đã xét) và truy hồi ngược lại (traceback) được đường đi bằng thuộc tính parent để trả về đúng định dạng.

+ Thắng: (DFS Implementation) Viết thuật toán DFS (Tìm kiếm theo chiều sâu).

+ Cường: (A Scaffold & Dummy Heuristic) Xây dựng bộ khung thuật toán A* (A-Star) với PriorityQueue (heapq). Tạm thời code hàm Heuristic trả về giá trị cố định h(n) = 0 (lúc này A* sẽ hoạt động y hệt thuật toán UCS). Đảm bảo khung A* chạy không bị lỗi cú pháp.

+ Đăng: chuẩn hoá API, Output cho từng file, sinh testcase, so sánh kết quả các thuật toán, đưa ra hướng đi tiếp theo

## Giai đoạn 2: Tối ưu Tìm kiếm bằng Heuristic (Informed Search)

  Mục tiêu: Giúp thuật toán A* chạy nhanh hơn, vượt qua các map cỡ trung bình nhờ vào việc "đánh hơi" được
  đường đi.

  + Nhi (Basic Box Heuristic): Tích hợp vào khung A* hàm Heuristic cơ bản: Tính tổng khoảng cách Manhattan từ mỗi Thùng (Box) đến Đích (Storage) gần nó nhất.

  + Thắng (Advanced Assignment Heuristic): Làm Heuristic nâng cao: Ghép cặp 1-1 (1 Thùng - 1 Đích). Code thuật toán Tham lam (Greedy) để ghép cặp sao cho tổng khoảng cách các cặp là nhỏ nhất. Cần xử lý logic cho trường hợp số Đích nhiều hơn số Thùng.

  + Cường (Robot Heuristic): Viết Heuristic phụ trợ: Tính thêm khoảng cách từ các Robot đang rảnh rỗi đến các Thùng chưa được đưa vào đích. Việc này rất quan trọng với các map Multi-Agent (nhiều robot) để điều phối robot tiếp cận công việc nhanh nhất.

  + Đăng (Tuning & Weighted A): Chạy thử nghiệm kết hợp các hàm Heuristic. Code thêm biến số W (Weighted A*: f(n) = g(n) + W × h(n)). Test thử các giá trị W.