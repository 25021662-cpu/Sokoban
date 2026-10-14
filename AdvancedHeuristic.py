def manhattan_distance(box, goal):
    return abs(box[0] - goal[0]) + abs(box[1] - goal[1])


def advanced_heuristic(state):
    boxes = sorted(state.boxes)
    goals = sorted(state.storage)

    if not boxes:
        return 0

    if len(goals) < len(boxes):
        return float("inf")

    pairs = []

    # Tinh khoang cach moi thung voi moi dich
    for i, box in enumerate(boxes):
        for j, goal in enumerate(goals):
            d = manhattan_distance(box, goal)
            pairs.append((d, i, j))

    # Sap xep theo khoang cach tang dan
    pairs.sort()

    used_boxes = set()
    used_goals = set()
    total = 0

    # Greedy: chon cap gan nhat chua duoc ghep
    for distance, i, j in pairs:
        if i in used_boxes or j in used_goals:
            continue

        used_boxes.add(i)
        used_goals.add(j)
        total += distance

        if len(used_boxes) == len(boxes):
            break

    return total