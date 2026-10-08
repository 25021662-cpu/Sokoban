from sokoban import SokobanState

def parse_level(file_path):
    """
    Hàm đọc map từ file text và chuyển đổi thành đối tượng SokobanState.
    Hỗ trợ cả 2 định dạng:
    1. Định dạng của AI-Multi-Agent: tường '+', robot '0-9', thùng 'A-Z', đích 'a-z'
    2. Định dạng Sokoban chuẩn: tường '#', robot '@', đích '.', thùng '$', 
       thùng trên đích '*', robot trên đích '+'
    """
    with open(file_path, 'r', encoding='utf-8') as f:
        raw_lines = f.readlines()
        
    map_lines = []
    for line in raw_lines:
        clean_line = line.rstrip('\n')
        # Bỏ qua dòng cấu hình màu của Multi-Agent
        if not clean_line or ':' in clean_line:
            continue
        map_lines.append(clean_line)
        
    height = len(map_lines)
    width = max(len(line) for line in map_lines) if height > 0 else 0
    
    robots_dict = {}
    boxes = set()
    storage = set()
    obstacles = set()
    
    # Biến phụ để đếm số lượng robot trong trường hợp map chuẩn dùng '@'
    robot_count = 0
    
    for y, line in enumerate(map_lines):
        for x, char in enumerate(line):
            # 1. Đọc Tường (Obstacles)
            if char == '+' or char == '#':
                # '+' là tường ở map Multi-Agent
                # Nhưng lưu ý: '+' cũng là Robot trên đích ở map chuẩn Sokoban
                # Ta phân biệt: nếu map có chữ '@' hoặc '$' thì nó là map chuẩn.
                # Tuy nhiên để an toàn, check nếu map chuẩn thì '+' là robot+storage.
                pass # Sẽ xử lý gộp bên dưới
                
            # --- XỬ LÝ MAP MULTI-AGENT CŨ ---
            if char.isdigit():
                robots_dict[int(char)] = (x, y)
            elif char.isalpha() and char.isupper():
                boxes.add((x, y))
            elif char.isalpha() and char.islower():
                storage.add((x, y))
                
            # --- XỬ LÝ MAP SOKOBAN CHUẨN (CÓ CHỒNG CHÉO KÝ HIỆU) ---
            elif char == '#':
                obstacles.add((x, y))
            elif char == '$':   # Thùng
                boxes.add((x, y))
            elif char == '.':   # Đích
                storage.add((x, y))
            elif char == '*':   # Thùng đang nằm trên Đích (Đè lên nhau)
                boxes.add((x, y))
                storage.add((x, y))
            elif char == '@':   # Robot
                robots_dict[robot_count] = (x, y)
                robot_count += 1
            elif char == '+':   
                # Ở map chuẩn: '+' là Robot nằm trên Đích
                # Ở map Multi-Agent: '+' là Tường
                # Để suy luận, ta xem map này có chứa ký tự '#' không. Nếu map không có '#' nào,
                # khả năng cao '+' đang được dùng làm tường.
                # Do đó, ta tạm lưu vào obstacles. Nếu là map chuẩn, người dùng nên dùng '#' làm tường.
                obstacles.add((x, y))

    # Nếu người dùng muốn ép ký hiệu '+' là Robot trên Đích (Map chuẩn),
    # họ phải tự đảm bảo Tường được vẽ bằng ký tự '#'.
    # Để chắc chắn, chúng ta duyệt lại:
    is_standard_map = any('#' in line for line in map_lines)
    if is_standard_map:
        obstacles.clear()
        for y, line in enumerate(map_lines):
            for x, char in enumerate(line):
                if char == '#':
                    obstacles.add((x, y))
                elif char == '+': # Robot đang đứng trên đích
                    robots_dict[robot_count] = (x, y)
                    robot_count += 1
                    storage.add((x, y))
                
    robots = tuple(pos for idx, pos in sorted(robots_dict.items()))
    
    return SokobanState(
        action="START",
        gval=0,
        parent=None,
        width=width,
        height=height,
        robots=robots,
        boxes=frozenset(boxes),
        storage=frozenset(storage),
        obstacles=frozenset(obstacles)
    )
