# Basic searching algorithms

# Class for each node in the grid
class Node:
    def __init__(self, row, col, is_obs):
        self.row = row  # coordinate
        self.col = col  # coordinate
        self.is_obs = is_obs  # obstacle?
        self.cost = None  # total cost (depend on the algorithm)
        self.parent = None  # previous node
        self.color = None


def bfs(grid, start, goal):
    '''Return a path found by BFS alogirhm 
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
    >>> bfs_path, bfs_steps = bfs(grid, start, goal)
    It takes 10 steps to find a path using BFS
    >>> bfs_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    '''
    ### YOUR CODE HERE ###

    row = 0

    cells = []
    for u in grid:
        nodes = []
        col = 0
        for i in u:
            node = Node(row, col, i)
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

    while len(Q) > 0:
        u = Q.pop(0)
        count = 0
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

            if cell.color == "White":
                steps += 1
                cell.color = "Gray"
                cell.cost = u.cost + 1
                cell.parent = u

                Q.append(cell)
                if cell.row == goal[0] and cell.col == goal[1]:
                    count += 1
        u.color = "Black"

        if count == 1:
            path.append([u.row, u.col])
            break

    if len(path) > 0:

        steps += 1
        while cells[path[-1][0]][path[-1][1]].row != start[0] or cells[path[-1][0]][path[-1][1]].col != start[1]:
            way = cells[path[-1][0]][path[-1][1]].parent
            path.append([way.row, way.col])

        path.reverse()
        path.append(goal)

    if goal in path:
        found = True
    else:
        found = False

    if found:
        print(f"It takes {steps} steps to find a path using BFS")
    else:
        print("No path found")
    return path, steps


def dfs(grid, start, goal):
    '''Return a path found by DFS alogirhm 
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
    >>> dfs_path, dfs_steps = dfs(grid, start, goal)
    It takes 9 steps to find a path using DFS
    >>> dfs_path
    [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2], [2, 3], [3, 3], [3, 2], [3, 1]]
    '''
    ### YOUR CODE HERE ###

    row = 0

    cells = []
    for u in grid:
        nodes = []
        col = 0
        for i in u:
            node = Node(row, col, i)
            node.color = "White"
            node.cost = float("inf")
            node.parent = None
            nodes.append(node)
            col += 1
        cells.append(nodes)
        row += 1

    def isvalid(row, col):

        # if the cell is already visited
        if cells[row][col].color == "Black":
            return False

        # otherwise, It can be visited
        return True

    # initialize the stack of pairs and push the starting cell into it
    st = [cells[start[0]][start[1]]]
    steps = 0
    path = []
    # Iterate until the stack is not empty

    while len(st) > 0:
        # Pop the top pair
        curr = st[len(st) - 1]
        count = 0
        st.remove(st[len(st) - 1])
        steps += 1
        row = curr.row
        col = curr.col
        #print("loop")

        # Mark the current
        # cell as visited
        cells[row][col].color = "Black"

        # 1
        if len(cells) > row - 1 >= 0:
            if cells[row - 1][col].is_obs == 0 and isvalid(row - 1, col):
                cells[row - 1][col].parent = curr
                st.append(cells[row - 1][col])
                if row - 1 == goal[0] and col == goal[1]:
                    break
        # 2
        if len(cells[0]) > col - 1 >= 0:
            if cells[row][col - 1].is_obs == 0 and isvalid(row, col - 1):
                cells[row][col - 1].parent = curr
                st.append(cells[row][col - 1])
                if row == goal[0] and col - 1 == goal[1]:
                    break
        # 3
        if len(cells) > row + 1 >= 0:
            if cells[row + 1][col].is_obs == 0 and isvalid(row + 1, col):
                cells[row + 1][col].parent = curr
                st.append(cells[row + 1][col])
                if row + 1 == goal[0] and col == goal[1]:
                    break
        # 4
        if len(cells[0]) > col + 1 >= 0:
            #print("inner loop")
            if cells[row][col + 1].is_obs == 0 and isvalid(row, col + 1):
                #print("inner inner loop")
                cells[row][col + 1].parent = curr
                st.append(cells[row][col + 1])
                if row == goal[0] and col + 1 == goal[1]:
                    break



    #print(st)
    if len(st) > 0:
        path.append([st[-1].row, st[-1].col])

        while cells[path[-1][0]][path[-1][1]].row != start[0] or cells[path[-1][0]][path[-1][1]].col != start[1]:
            way = cells[path[-1][0]][path[-1][1]].parent
            path.append([way.row, way.col])

        path.reverse()
    steps += 1
    # path.append(goal)

    if goal in path:
        found = True
    else:
        found = False

    if found:
        print(f"It takes {steps} steps to find a path using DFS")
    else:
        print("No path found")
    return path, steps


# Doctest
if __name__ == "__main__":
    # load doc test
    from doctest import testmod, run_docstring_examples

    # Test all the functions
    testmod()
