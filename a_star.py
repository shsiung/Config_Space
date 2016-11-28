import math

def dist(m, n):
    """Returns the squared Euclidean distance between m and n"""
    dx, dy = n[0] - m[0], n[1] - m[1]
    return math.sqrt(dx * dx + dy * dy)

def a_star(start, goal, all_node):
    closedSet = []
    openSet = [start]
    cameFrom = []

    for node in all_node:
        node.gScore = 100000
    start.gScore = 0

    for node in all_node:
        node.fScore = 100000

    start.fScore = dist(start.node,goal.node)
    start.cameFrom = []

    while openSet != []:
        openSet.sort(key=lambda x: x.fScore)
        current_node = openSet.pop(0)
        if current_node == goal:
            return reconstruct_path(current_node.cameFrom, current_node)
        else:
            closedSet += [current_node]
            for neighbor in current_node.neighbor:
                if neighbor in closedSet:
                    continue
                else:
                    tentative_gScore = current_node.gScore + dist(current_node.node, neighbor.node)
                    if neighbor not in openSet:
                        openSet += [neighbor]
                    elif tentative_gScore >= neighbor.gScore:
                        continue
                    neighbor.cameFrom = current_node
                    neighbor.gScore = tentative_gScore
                    neighbor.fScore = neighbor.gScore + dist(neighbor.node,goal.node)

    return False

def reconstruct_path(cameFrom, current):
    total_path = [current]
    while current.cameFrom != []:
        current = current.cameFrom
        total_path += [current]
    return total_path