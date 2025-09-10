import heapq

class PriorityQueue:
    def __init__(self):
        self.elements = []

    def push(self, item, priority):
        heapq.heappush(self.elements, (priority, item))

    def pop(self):
        return heapq.heappop(self.elements)[1]

    def empty(self):
        return not self.elements

def get_neighbors(map_grid, current):
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    neighbors = []
    x, y = current
    height = len(map_grid)
    width = len(map_grid[0])
    for dx, dy in directions:
        nx, ny = x + dx, y + dy
        if 0 <= ny < height and 0 <= nx < width:
            if map_grid[ny][nx] == 0:
                neighbors.append((nx, ny))
    return neighbors

def movement_cost(a, b):
    return 1  # Constant cost for cardinal directions

def reconstruct_path(came_from, start, goal):
    path = [goal]
    while goal != start:
        goal = came_from[goal]
        path.append(goal)
    path.reverse()
    return path

def dijkstra(map_grid, start, goal):
    frontier = PriorityQueue()
    frontier.push(start, 0)
    came_from = {}
    cost_so_far = {start: 0}

    while not frontier.empty():
        current = frontier.pop()
        if current == goal:
            break

        for neighbor in get_neighbors(map_grid, current):
            new_cost = cost_so_far[current] + movement_cost(current, neighbor)
            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                frontier.push(neighbor, new_cost)
                came_from[neighbor] = current

    if goal in came_from:
        return reconstruct_path(came_from, start, goal)
    else:
        return []

