# Basic searching algorithms

# Class for each node in the grid
class Node:
    def __init__(self, row, col, is_obs, h):
        self.row = row  # coordinate
        self.col = col  # coordinate
        self.is_obs = is_obs  # obstacle?
        self.g = None  # cost to come (previous g + moving cost)
        self.h = h  # heuristic
        self.cost = None  # total cost (depend on the algorithm)
        self.parent = None  # previous node
        self.color = None


def dijkstra(grid, start, goal):
    '''Return a path found by Dijkstra alogirhm
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node),
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution,
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> dij_path, dij_steps = dijkstra(grid, start, goal)
    It takes 10 steps to find a path using Dijkstra
    >>> dij_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    '''
    ### YOUR CODE HERE ###

    row = 0

    cells = []
    for u in grid:
        nodes = []
        col = 0
        for i in u:
            node = Node(row, col, i, None)
            node.color = "White"
            node.cost = float("inf")
            node.parent = None
            nodes.append(node)
            col += 1
        cells.append(nodes)
        row += 1

    cells[start[0]][start[1]].color = "Gray"
    cells[start[0]][start[1]].cost = 0
    Q = [cells[start[0]][start[1]]]
    steps = 0
    path = []

    def returnmindis(Q):
        mini = Q[0]
        for c in Q:
            if mini.cost > c.cost:
                mini = c
        return mini

    while len(Q) > 0:
        u = returnmindis(Q)
        Q.remove(u)
        steps += 1
        count = 0
        adj = []
        if u.row == goal[0] and u.col == goal[1]:
            count += 1

        if len(cells[0]) > u.col + 1 >= 0:
            if cells[u.row][u.col + 1].is_obs == 0:
                adj.append(cells[u.row][u.col + 1])

        if len(cells) > u.row + 1 >= 0:
            if cells[u.row + 1][u.col].is_obs == 0:
                adj.append(cells[u.row + 1][u.col])

        if len(cells[0]) > u.col - 1 >= 0:
            if cells[u.row][u.col - 1].is_obs == 0:
                adj.append(cells[u.row][u.col - 1])

        if len(cells) > u.row - 1 >= 0:
            if cells[u.row - 1][u.col].is_obs == 0:
                adj.append(cells[u.row - 1][u.col])

        for cell in adj:

            if cell.color != "Black":
                if cell.cost > u.cost + 1:
                    cell.cost = u.cost + 1
                    cell.parent = u

                    if cell.color != "Gray":
                        Q.append(cell)

                    cell.color = "Gray"
        u.color = "Black"

        if count == 1:
            path.append([u.row, u.col])
            break

    if len(path) > 0:

        while cells[path[-1][0]][path[-1][1]].row != start[0] or cells[path[-1][0]][path[-1][1]].col != start[1]:
            way = cells[path[-1][0]][path[-1][1]].parent
            path.append([way.row, way.col])

        path.reverse()

    if goal in path:
        found = True
    else:
        found = False

    if found:
        print(f"It takes {steps} steps to find a path using Dijkstra")
    else:
        print("No path found")
    return path, steps


def astar(grid, start, goal):
    '''Return a path found by A* alogirhm
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node),
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution,
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> astar_path, astar_steps = astar(grid, start, goal)
    It takes 7 steps to find a path using A*
    >>> astar_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    '''
    ### YOUR CODE HERE ###
    row = 0

    cells = []
    for u in grid:
        nodes = []
        col = 0
        for i in u:
            node = Node(row, col, i, None)
            node.color = "White"
            node.g = float("inf")
            node.h = abs(row - goal[0]) + abs(col - goal[1])
            node.cost = node.g + node.h
            node.parent = None
            nodes.append(node)
            col += 1
        cells.append(nodes)
        row += 1

    cells[start[0]][start[1]].color = "Gray"
    cells[start[0]][start[1]].g = 0
    cells[start[0]][start[1]].cost = cells[start[0]][start[1]].g + cells[start[0]][start[1]].h
    Q = [cells[start[0]][start[1]]]
    steps = 0
    path = []

    def returnmindis(Q):
        mini = Q[0]
        for c in Q:
            if mini.cost > c.cost:
                mini = c
        return mini

    while len(Q) > 0:
        u = returnmindis(Q)
        Q.remove(u)
        steps += 1
        count = 0
        if u.row == goal[0] and u.col == goal[1]:
            count += 1
        adj = []

        if len(cells[0]) > u.col + 1 >= 0:
            if cells[u.row][u.col + 1].is_obs == 0:
                adj.append(cells[u.row][u.col + 1])

        if len(cells) > u.row + 1 >= 0:
            if cells[u.row + 1][u.col].is_obs == 0:
                adj.append(cells[u.row + 1][u.col])

        if len(cells[0]) > u.col - 1 >= 0:
            if cells[u.row][u.col - 1].is_obs == 0:
                adj.append(cells[u.row][u.col - 1])

        if len(cells) > u.row - 1 >= 0:
            if cells[u.row - 1][u.col].is_obs == 0:
                adj.append(cells[u.row - 1][u.col])

        for cell in adj:

            if cell.color != "Black":
                if cell.g > u.g + 1:
                    cell.g = u.g + 1
                    cell.cost = cell.g + cell.h
                    cell.parent = u

                    if cell.color != "Gray":
                        Q.append(cell)

                    cell.color = "Gray"

        u.color = "Black"

        if count == 1:
            path.append([u.row, u.col])
            break

    if len(path) > 0:

        while cells[path[-1][0]][path[-1][1]].row != start[0] or cells[path[-1][0]][path[-1][1]].col != start[1]:
            way = cells[path[-1][0]][path[-1][1]].parent
            path.append([way.row, way.col])

        path.reverse()

    if goal in path:
        found = True
    else:
        found = False

    if found:
        print(f"It takes {steps} steps to find a path using A*")
    else:
        print("No path found")
    return path, steps


# Doctest
if __name__ == "__main__":
    # load doc test
    from doctest import testmod, run_docstring_examples

    # Test all the functions
    testmod()
