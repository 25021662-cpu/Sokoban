def map_err(state):
    """
    Tìm tất cả các vị trí "deadlock" (vị trí chết) tĩnh trên bản đồ.
    Nếu hộp bị đẩy vào các vị trí này, màn chơi chắc chắn không thể hoàn thành.
    Bao gồm Corner Deadlocks (góc chết) và Edge Deadlocks (cạnh chết).
    
    :param state: Đối tượng trạng thái (SokobanState) chứa thông tin bản đồ.
    :return: Một set chứa các tọa độ (x, y) là deadlock.
    """
    width = state.width
    height = state.height
    obstacles = state.obstacles
    storage = state.storage

    def is_wall(cx, cy):
        return cx < 0 or cx >= width or cy < 0 or cy >= height or (cx, cy) in obstacles

    deadlocks = set()

    # 1. Phát hiện Corner Deadlocks (Góc chết)
    for x in range(width):
        for y in range(height):
            if (x, y) in obstacles or (x, y) in storage:
                continue
            
            # Kiểm tra xem có bị bao bọc bởi tường ở cả 2 hướng vuông góc không
            up = is_wall(x, y - 1)
            down = is_wall(x, y + 1)
            left = is_wall(x - 1, y)
            right = is_wall(x + 1, y)
            
            if (up or down) and (left or right):
                deadlocks.add((x, y))

    # 2. Phát hiện Edge Deadlocks (Cạnh chết)
    # Lặp lại vài lần để các deadlock cạnh liền kề có thể phát hiện dựa trên nhau
    changed = True
    while changed:
        changed = False
        for x in range(width):
            for y in range(height):
                if (x, y) in obstacles or (x, y) in storage or (x, y) in deadlocks:
                    continue
                
                # Quét cạnh ngang (Tường ở TRÊN)
                if is_wall(x, y - 1):
                    lx = x
                    while not is_wall(lx, y) and not (lx, y) in deadlocks and not (lx, y) in storage and is_wall(lx, y - 1):
                        lx -= 1
                    left_closed = is_wall(lx, y) or (lx, y) in deadlocks
                    
                    rx = x
                    while not is_wall(rx, y) and not (rx, y) in deadlocks and not (rx, y) in storage and is_wall(rx, y - 1):
                        rx += 1
                    right_closed = is_wall(rx, y) or (rx, y) in deadlocks
                    
                    if left_closed and right_closed:
                        for i in range(lx + 1, rx):
                            if (i, y) not in deadlocks:
                                deadlocks.add((i, y))
                                changed = True

                # Quét cạnh ngang (Tường ở DƯỚI)
                if is_wall(x, y + 1):
                    lx = x
                    while not is_wall(lx, y) and not (lx, y) in deadlocks and not (lx, y) in storage and is_wall(lx, y + 1):
                        lx -= 1
                    left_closed = is_wall(lx, y) or (lx, y) in deadlocks
                    
                    rx = x
                    while not is_wall(rx, y) and not (rx, y) in deadlocks and not (rx, y) in storage and is_wall(rx, y + 1):
                        rx += 1
                    right_closed = is_wall(rx, y) or (rx, y) in deadlocks
                    
                    if left_closed and right_closed:
                        for i in range(lx + 1, rx):
                            if (i, y) not in deadlocks:
                                deadlocks.add((i, y))
                                changed = True

                # Quét cạnh dọc (Tường ở TRÁI)
                if is_wall(x - 1, y):
                    uy = y
                    while not is_wall(x, uy) and not (x, uy) in deadlocks and not (x, uy) in storage and is_wall(x - 1, uy):
                        uy -= 1
                    up_closed = is_wall(x, uy) or (x, uy) in deadlocks
                    
                    dy = y
                    while not is_wall(x, dy) and not (x, dy) in deadlocks and not (x, dy) in storage and is_wall(x - 1, dy):
                        dy += 1
                    down_closed = is_wall(x, dy) or (x, dy) in deadlocks
                    
                    if up_closed and down_closed:
                        for i in range(uy + 1, dy):
                            if (x, i) not in deadlocks:
                                deadlocks.add((x, i))
                                changed = True

                # Quét cạnh dọc (Tường ở PHẢI)
                if is_wall(x + 1, y):
                    uy = y
                    while not is_wall(x, uy) and not (x, uy) in deadlocks and not (x, uy) in storage and is_wall(x + 1, uy):
                        uy -= 1
                    up_closed = is_wall(x, uy) or (x, uy) in deadlocks
                    
                    dy = y
                    while not is_wall(x, dy) and not (x, dy) in deadlocks and not (x, dy) in storage and is_wall(x + 1, dy):
                        dy += 1
                    down_closed = is_wall(x, dy) or (x, dy) in deadlocks
                    
                    if up_closed and down_closed:
                        for i in range(uy + 1, dy):
                            if (x, i) not in deadlocks:
                                deadlocks.add((x, i))
                                changed = True

    return frozenset(deadlocks)


def is_2x2_deadlock(state):
    """
    Phát hiện các khối 2x2 bị kẹt cứng (Freeze Deadlock).
    Nếu 4 ô tạo thành hình vuông 2x2 chỉ chứa Tường và Hộp,
    thì không hộp nào trong khối đó có thể di chuyển được nữa.
    Nếu có ít nhất 1 hộp trong khối đó không nằm trên đích -> Deadlock vĩnh viễn.
    """
    boxes = state.boxes
    obstacles = state.obstacles
    storage = state.storage
    width = state.width
    height = state.height

    def is_solid(x, y):
        if x < 0 or x >= width or y < 0 or y >= height:
            return True
        return (x, y) in obstacles or (x, y) in boxes

    def check_2x2(x, y):
        # Kiểm tra 4 ô trong hình vuông 2x2 có bị khối cứng che lấp hết không
        if not is_solid(x, y): return False
        if not is_solid(x+1, y): return False
        if not is_solid(x, y+1): return False
        if not is_solid(x+1, y+1): return False
        
        # Nếu đã là khối cứng, kiểm tra xem có hộp nào ĐANG NẰM LỆCH ĐÍCH không?
        for cx, cy in [(x, y), (x+1, y), (x, y+1), (x+1, y+1)]:
            if (cx, cy) in boxes and (cx, cy) not in storage:
                return True # Phát hiện kẹt!
        return False

    # Chỉ cần dò xung quanh các hộp hiện tại để tối ưu tốc độ
    for bx, by in boxes:
        # Hộp (bx, by) có thể là 1 trong 4 góc của hình vuông 2x2
        if check_2x2(bx, by) or \
           check_2x2(bx-1, by) or \
           check_2x2(bx, by-1) or \
           check_2x2(bx-1, by-1):
            return True

    return False

def is_deadlock(state, deadlocks):
    """
    Kiểm tra xem trạng thái hiện tại có bị rơi vào bất kỳ Deadlock nào không.
    :param state: Trạng thái hiện tại
    :param deadlocks: Tập hợp tọa độ deadlock tĩnh (từ map_err)
    :return: True nếu vô nghiệm, ngược lại False.
    """
    # 1. Deadlock tĩnh (Hộp bị đẩy vào góc/cạnh chết)
    for box in state.boxes:
        if box in deadlocks:
            return True
            
    # 2. Deadlock động (Các hộp dính vào nhau hoặc dính vào tường tạo thành khối 2x2)
    if is_2x2_deadlock(state):
        return True
        
    return False
