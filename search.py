import heapq

# search.py
# ---------------
# Licensing Information:  You are free to use or extend this projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to the University of Illinois at Urbana-Champaign
#
# Created by Michael Abir (abir2@illinois.edu) on 08/28/2018

"""
This is the main entry point for MP1. You should only modify code
within this file -- the unrevised staff files will be used for all other
files and classes when code is run, so be careful to not modify anything else.
"""
# Search should return the path.
# The path should be a list of tuples in the form (row, col) that correspond
# to the positions of the path taken by your search algorithm.
# maze is a Maze object based on the maze from the file specified by input filename
# searchMethod is the search method specified by --method flag (bfs,dfs,astar,astar_multi,fast)

class PointData:
    x = 0
    y = 0
    g = 0
    h = 0
    f = 0


def search(maze, searchMethod):
    return {
        "bfs": bfs,
        "astar": astar,
        "astar_corner": astar_corner,
        "astar_multi": astar_multi,
        "fast": fast,
    }.get(searchMethod)(maze)

def bfs(maze):
    """
    Runs BFS for part 1 of the assignment.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """
    # TODO: Write your code here
    q = []
    visited = {}
    q.append(maze.getStart())
    pairs = {}  # maps from 2nd step -> first step

    wonSpot = None
    while len(q) > 0:
        curr = q.pop(0)
        if maze.isObjective(curr[0], curr[1]):
            wonSpot = curr
            break
        neighbors = maze.getNeighbors(curr[0], curr[1])
        for n in neighbors:
            if n not in visited:
                # print("appending", n)
                visited[n] = True
                q.append(n)
                pairs[n] = curr

    # print("done", wonSpot)
    # now reconstruct path
    curr = wonSpot
    path = []
    while curr != maze.getStart():
        path.append(curr)
        curr = pairs[curr]
    path.append(curr)
    path.reverse()

    return path

def manhattanDist(pt1, pt2):
    return abs(pt2[0] - pt1[0]) + abs(pt2[1] - pt1[1])

def astarHelper(maze, start, goal):
    # f = g(=path len) + mandist
    heap = []
    visited = {}
    pairs = {}
    '''
    curr = PointData()
    curr.x = maze.getStart()[0]
    curr.y = maze.getStart()[1]
    curr.g = 0
    curr.h = manhattanDist(maze.getStart(), endGoal)
    curr.f = curr.h + curr.g
    '''
    # tuple: (f, g, h, x, y)
    curr = (manhattanDist(start, goal), 0, manhattanDist(start, goal), start[0],
            start[1])
    heapq.heappush(heap, curr)

    wonSpot = None
    while len(heap) > 0:
        curr = heapq.heappop(heap)
        if goal == (curr[3], curr[4]):
            wonSpot = curr
            break
        neighbors = maze.getNeighbors(curr[3], curr[4])
        for n in neighbors:
            newN = (manhattanDist(n, goal) + curr[1] + 1, curr[1] + 1, manhattanDist(n, goal), n[0], n[1])
            '''
            newN.x = n[0]
            newN.y = n[1]
            newN.g = curr.g + 1
            newN.h = manhattanDist(n, endGoal)
            newN.f = newN.h + newN.g
            '''
            if (newN[3], newN[4]) not in visited:
                # print("appending", n)
                visited[(newN[3], newN[4])] = True
                heapq.heappush(heap, newN)
                pairs[newN] = curr

    curr = wonSpot
    path = []
    while (curr[3], curr[4]) != start:
        path.append((curr[3], curr[4]))
        curr = pairs[curr]
    path.append((curr[3], curr[4]))
    path.reverse()
    return path

def astar(maze):
    """
    Runs A star for part 1 of the assignment.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """
    # TODO: Write your code here
    return astarHelper(maze, maze.getStart(), maze.getObjectives()[0])
    '''
    # f = g(=path len) + mandist
    endGoal = maze.getObjectives()[0]
    heap = []
    visited = {}
    pairs = {}
    #tuple: (f, g, h, x, y)
    curr = (manhattanDist(maze.getStart(), endGoal), 0, manhattanDist(maze.getStart(), endGoal), maze.getStart()[0], maze.getStart()[1])
    heapq.heappush(heap, curr)

    wonSpot = None
    while len(heap) > 0:
        curr = heapq.heappop(heap)
        if maze.isObjective(curr[3], curr[4]):
            wonSpot = curr
            break
        neighbors = maze.getNeighbors(curr[3], curr[4])
        for n in neighbors:
            newN = (manhattanDist(n, endGoal) + curr[1] + 1, curr[1] + 1, manhattanDist(n, endGoal), n[0], n[1])
            if (newN[3], newN[4]) not in visited:
                # print("appending", n)
                visited[(newN[3], newN[4])] = True
                heapq.heappush(heap, newN)
                pairs[newN] = curr

    curr = wonSpot
    path = []
    while (curr[3], curr[4]) != maze.getStart():
        path.append((curr[3], curr[4]))
        curr = pairs[curr]
    path.append((curr[3], curr[4]))
    path.reverse()
    return path
    '''

def astar_corner(maze):
    """
    Runs A star for part 2 of the assignment in the case where there are four corner objectives.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
        """
    # TODO: Write your code here
    #find correct order of goals
    goals = maze.getObjectives()

    #create path between them
    path = []
    start = maze.getStart()
    for goal in goals:
        path += astarHelper(maze, start, goal)
        start = goal
    return path

def astar_multi(maze):
    """
    Runs A star for part 3 of the assignment in the case where there are
    multiple objectives.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """
    # TODO: Write your code here
    return []


def fast(maze):
    """
    Runs suboptimal search algorithm for part 4.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """
    # TODO: Write your code here
    return []
