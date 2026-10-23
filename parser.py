from sokoban import SokobanState

def parse_level(file_path):
    """
    Parse chuỗi biểu diễn Sokoban thành đối tượng SokobanState.
    Nếu không truyền state_class, hàm trả về một dict chứa các thuộc tính.
    """
    with open(file_path, 'r') as f:
        state_str = f.read()

    # Tách chuỗi thành các dòng và loại bỏ dòng trống đầu/cuối nếu có
    lines = [line.rstrip('\r') for line in state_str.split('\n') if line]
    
    # Loại bỏ viền trên cùng và dưới cùng
    inner_lines = lines[1:-1]
    
    height = len(inner_lines)
    # Loại bỏ ký tự viền '#' ở đầu và cuối mỗi dòng
    width = len(inner_lines[0]) - 2 if height > 0 else 0
    
    storage = []
    obstacles = []
    boxes = []
    robots_dict = {}  # Lưu robot theo chỉ số để sắp xếp đúng thứ tự (a/A -> 0, b/B -> 1, ...)

    for y, line in enumerate(inner_lines):
        # Cắt bỏ '#' ở hai đầu
        content = line[1:-1]
        for x, char in enumerate(content):
            pos = (x, y)
            
            if char == '#':
                obstacles.append(pos)
            elif char == '.':
                storage.append(pos)
            elif char == '$':
                boxes.append(pos)
            elif char == '*':
                boxes.append(pos)
                storage.append(pos)
            elif 'a' <= char <= 'z':
                idx = ord(char) - ord('a')
                robots_dict[idx] = pos
            elif 'A' <= char <= 'Z':
                idx = ord(char) - ord('A')
                robots_dict[idx] = pos
                storage.append(pos)
            elif char == ' ':
                continue
                
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
