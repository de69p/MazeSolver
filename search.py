import heapq
import argparse


# Зчитування лабіринту з файлу
def read_maze(filename):
    with open(filename, 'r') as f:
        maze = [list(line.strip()) for line in f]
    return maze


# Манхеттенська відстань
def manhattan_distance(x1, y1, x2, y2):
    return abs(x1 - x2) + abs(y1 - y2)


# Реалізація A* пошуку
def a_star_search(start, goal, maze):
    open_list = [(manhattan_distance(start[0], start[1], goal[0], goal[1]), 0, start)]
    visited = set()
    cost_so_far = {start: 0}
    came_from = {start: None}

    while open_list:
        _, cost, current = heapq.heappop(open_list)
        if current in visited:
            continue
        visited.add(current)

        if current == goal:
            path = []
            while current is not None:
                path.append(current)
                current = came_from[current]
            path.reverse()
            return path, cost, len(visited)

        for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
            x, y = current
            next_state = (x + dx, y + dy)
            nx, ny = next_state
            if 0 <= nx < len(maze) and 0 <= ny < len(maze[0]) and maze[nx][ny] != '%' and next_state not in visited:
                new_cost = cost_so_far[current] + 1
                if next_state not in cost_so_far or new_cost < cost_so_far[next_state]:
                    cost_so_far[next_state] = new_cost
                    priority = new_cost + manhattan_distance(goal[0], goal[1], nx, ny)
                    heapq.heappush(open_list, (priority, new_cost, next_state))
                    came_from[next_state] = current


# Реалізація DFS
def dfs(start, goal, maze):
    stack = [start]
    visited = set()
    came_from = {start: None}

    while stack:
        current = stack.pop()
        if current in visited:
            continue
        visited.add(current)

        if current == goal:
            path = []
            while current is not None:
                path.append(current)
                current = came_from[current]
            path.reverse()
            return path, len(path) - 1, len(visited)

        for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
            x, y = current
            next_state = (x + dx, y + dy)
            nx, ny = next_state
            if 0 <= nx < len(maze) and 0 <= ny < len(maze[0]) and maze[nx][ny] != '%' and next_state not in visited:
                stack.append(next_state)
                came_from[next_state] = current


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Maze Solver')
    parser.add_argument('--method', type=str, required=True, help='Search method: dfs or astar')
    parser.add_argument('--heuristic', type=str, default=None, help='Heuristic function: manhattan or none')
    parser.add_argument('filename', type=str, help='Maze filename')
    args = parser.parse_args()

    maze = read_maze(args.filename)
    start = None
    goal = None

    for i in range(len(maze)):
        for j in range(len(maze[0])):
            if maze[i][j] == 'S':
                start = (i, j)
            elif maze[i][j] == 'G':
                goal = (i, j)

    if args.method == "astar":
        if args.heuristic == "manhattan":
            path, cost, expanded_nodes = a_star_search(start, goal, maze)
        else:
            print("Invalid heuristic")
            exit(1)
    elif args.method == "dfs":
        path, cost, expanded_nodes = dfs(start, goal, maze)
    else:
        print("Invalid method")
        exit(1)

    for x, y in path:
        maze[x][y] = 'x'
    maze[start[0]][start[1]] = 'S'
    maze[goal[0]][goal[1]] = 'G'

    for row in maze:
        print("".join(row))

    print(f"Solution cost: {cost}")
    print(f"Expanded nodes: {expanded_nodes}")
