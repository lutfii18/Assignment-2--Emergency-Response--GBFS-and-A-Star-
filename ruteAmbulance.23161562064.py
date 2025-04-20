import heapq
import time

grid = [
    ['S', '.', 'T', '.', 'H'],
    ['.', 'T', '.', '.', '.'],
    ['.', '.', '.', 'T', '.'],
]
start = (0, 0)
goal = (0, 4)

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def gbfs(start, goal, grid):
    queue = [(heuristic(start, goal), start)]
    came_from = {start: None}
    visited = set()
    node_count = 0

    while queue:
        _, current = heapq.heappop(queue)
        node_count += 1
        if current == goal:
            break
        visited.add(current)
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nx, ny = current[0] + dx, current[1] + dy
            next_node = (nx, ny)
            if 0 <= nx < len(grid) and 0 <= ny < len(grid[0]) and \
               grid[nx][ny] != 'T' and next_node not in visited:
                came_from[next_node] = current
                heapq.heappush(queue, (heuristic(next_node, goal), next_node))

    # Membuat path dari goal ke start
    path = []
    node = goal
    while node:
        path.append(node)
        node = came_from.get(node)
    path.reverse()

    # Jika path tidak ditemukan
    if not path or path[0] != start:
        return None, node_count
    
    return path, node_count

def a_star(start, goal, grid):
    queue = [(0, start)]
    came_from = {start: None}
    cost_so_far = {start: 0}
    node_count = 0

    while queue:
        _, current = heapq.heappop(queue)
        node_count += 1
        if current == goal:
            break
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nx, ny = current[0] + dx, current[1] + dy
            next_node = (nx, ny)
            if 0 <= nx < len(grid) and 0 <= ny < len(grid[0]) and grid[nx][ny] != 'T':
                new_cost = cost_so_far[current] + 1
                if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                    cost_so_far[next_node] = new_cost
                    priority = new_cost + heuristic(next_node, goal)
                    heapq.heappush(queue, (priority, next_node))
                    came_from[next_node] = current

    # Membuat path dari goal ke start
    path = []
    node = goal
    while node:
        path.append(node)
        node = came_from.get(node)
    path.reverse()

    # Jika path tidak ditemukan
    if not path or path[0] != start:
        return None, node_count

    return path, node_count

# Perbandingan antara GBFS dan A*
start_time = time.time()
gbfs_path, gbfs_nodes = gbfs(start, goal, grid)
gbfs_time = (time.time() - start_time) * 1000

start_time = time.time()
astar_path, astar_nodes = a_star(start, goal, grid)
astar_time = (time.time() - start_time) * 1000

print("\n[Assignment 2] Ambulance Grid")
if gbfs_path:
    print("GBFS Path:", gbfs_path, f"Time: {gbfs_time:.3f}ms, Nodes: {gbfs_nodes}")
else:
    print("GBFS: Jalur tidak ditemukan.")

if astar_path:
    print("A*   Path:", astar_path, f"Time: {astar_time:.3f}ms, Nodes: {astar_nodes}")
else:
    print("A*: Jalur tidak ditemukan.")
