# Tất cả thành viên cần hiểu rõ cách các API hoạt động, 1 State lưu những gì, mình đang làm gì, ... Cuối mỗi giai đoạn họp nhóm review lại toàn bộ code

## Giai đoạn 1: Xây dựng nền móng & Thuật toán cơ bản (Blind Search)

Mục tiêu: Hiểu rõ framework, dựng được luồng chạy cơ bản, vượt qua các test dễ nhất bằng thuật toán tìm kiếm mù, việc có 2-3 thuật toán mù trong giai đoạn đầu sẽ giúp team dễ dàng đối chiếu kết quả (cross-check) xem code sinh trạng thái có bị lỗi ở đâu không.

+ Nhi: (BFS Implementation) Viết thuật toán BFS (Tìm kiếm theo chiều rộng). Đảm bảo quản lý tốt tập hợp explored (các trạng thái đã xét) và truy hồi ngược lại (traceback) được đường đi bằng thuộc tính parent để trả về đúng định dạng.

+ Thắng: (DFS Implementation) Viết thuật toán DFS (Tìm kiếm theo chiều sâu).

+ Cường: (A Scaffold & Dummy Heuristic) Xây dựng bộ khung thuật toán A* (A-Star) với PriorityQueue (heapq). Tạm thời code hàm Heuristic trả về giá trị cố định h(n) = 0 (lúc này A* sẽ hoạt động y hệt thuật toán UCS). Đảm bảo khung A* chạy không bị lỗi cú pháp.

+ Đăng: chuẩn hoá API, Output cho từng file, sinh testcase, so sánh kết quả các thuật toán, đưa ra hướng đi tiếp theo