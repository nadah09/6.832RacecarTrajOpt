from generate_track import OccupancyGrid

def reconstruct_path(came_from, start, goal):
    current = goal
    path = [current]
    while current != start:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path

def a_star(occ_grid, start, end):
    queue = [start]
    parents = {start: None}
    totalCost = {start: 0}

    while queue:
        curr = queue.pop(0)
        if curr == end:
            break
        for n in occ_grid.get_neighbors(curr):
            new_cost = totalCost[curr] + 1
            if n not in totalCost or new_cost < totalCost[n]:
                totalCost[n] = new_cost
                queue.append(n)
                parents[n] = curr
    
    path = reconstruct_path(parents, start, end)
    return path
