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
# searchMethod is the search method specified by --method flag (bfs,dfs,astar,astar_multi,extra)

def search(maze, searchMethod):
    return {
        "bfs": bfs,
        "astar": astar,
        "astar_corner": astar_corner,
        "astar_multi": astar_multi,
        "extra": extra,
    }.get(searchMethod)(maze)


def sanity_check(maze, path):
    """
    Runs check functions for part 0 of the assignment.

    @param maze: The maze to execute the search on.
    @param path: a list of tuples containing the coordinates of each state in the computed path

    @return bool: whether or not the path pass the sanity check
    """
    # TODO: Write your code here
    return False


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
    pairs = {} #maps from 2nd step -> first step

    wonSpot = None
    while len(q) > 0:
        curr = q.pop()
        visited[curr] = True
        if maze.isObjective(curr[0], curr[1]):
            wonSpot = curr
            break
        neighbors = maze.getNeighbors(curr[0], curr[1])
        for n in neighbors:
            if n not in visited:
                #print("appending", n)
                q.append(n)
                pairs[n] = curr

    #print("done", wonSpot)
    #now reconstruct path
    curr = wonSpot
    path = []
    while curr != maze.getStart():
        path.append(curr)
        curr = pairs[curr]
    path.append(curr)
    path.reverse()

    return path


def astar(maze):
    """
    Runs A star for part 1 of the assignment.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """
    # TODO: Write your code here
    return []

def astar_corner(maze):
    """
    Runs A star for part 2 of the assignment in the case where there are four corner objectives.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
        """
    # TODO: Write your code here
    return []

def astar_multi(maze):
    """
    Runs A star for part 3 of the assignment in the case where there are
    multiple objectives.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """
    # TODO: Write your code here
    return []


def extra(maze):
    """
    Runs suboptimal search algorithm for part 4.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """
    # TODO: Write your code here
    return []
