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

def manhattanDist(pt1, pt2):
    return abs(pt2[0] - pt1[0]) + abs(pt2[1] - pt1[1])


#graph def
class Node:
    value = 0
    coord = (0, 0)
    edge_list = []

    def __init__(self, in_value, in_coord, in_edge_list):
        self.value = in_value
        self.edge_list = in_edge_list
        self.coord = in_coord

    def get_info(self):
        print("Node with value", self.value, "and coord", self.coord)

class Edge:
    start = None
    end = None
    length = 0

    def __init__(self, in_start, in_end, in_length):
        self.start = in_start
        self.end = in_end
        self.length = in_length

    def __lt__(self, other):
        return self.length < other.length

    def __gt__(self, other):
        return self.length > other.length

    def get_info(self):
        print("Edge from", self.start, "to", self.end, "with length", self.length)

class Graph:
    node_list = {} #maps point -> node
    msts = {} #maps set of remaining goals -> distance

    def __init__(self, goals):
        #construct graph
        for goal in goals:
            #create edges for goal
            edges = []
            for g in goals:
                if g != goal:
                    e = Edge(goal, g, manhattanDist(goal, g))
                    edges.append(e)
                    #print("adding edge to", goal)
                    #e.get_info()

            n = Node(float('inf'), goal, edges)
            self.node_list[goal] = n

    #prims alg
    def mst(self, goalsLeft, start):
        if tuple(goalsLeft) in self.msts.keys():
            #print("branch 1")
            return self.msts[tuple(goalsLeft)]
        else:
            #print("branch 2")
            goalsLeft2 = goalsLeft.copy()
            edgeSum = 0
            #add prim alg code here
            edgeQueue = []
            currGoal = start
            while len(goalsLeft) > 0:
                #explore this node
                curr = self.node_list[currGoal]
                #add all edges to queue
                edges = curr.edge_list
                for e in edges:
                    if e.start in goalsLeft or e.end in goalsLeft:
                        heapq.heappush(edgeQueue, e)
                #mark this node as visited TODO: make more efficient later
                if currGoal in goalsLeft:
                    goalsLeft.remove(currGoal)
                #follow lowest edge to new node
                flag = False
                while flag is False and len(edgeQueue) > 0:
                    lowEdge = heapq.heappop(edgeQueue)
                    if lowEdge.start in goalsLeft:
                        currGoal = lowEdge.start
                        edgeSum += lowEdge.length
                        flag = True
                    elif lowEdge.end in goalsLeft:
                        currGoal = lowEdge.end
                        edgeSum += lowEdge.length
                    flag = True
                if flag is False:
                    #print("queue empty")
                    break

            #end prim code
            self.msts[tuple(goalsLeft2)] = edgeSum
            #print("esum:", edgeSum)
            return 0


#keeps track of previous minLen values so dont have to recalculate
minLens = {} #map set of remaining goals -> distance


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

def h(start, goals, h_type, graph = None):
    if h_type == "md":
        return manhattanDist(start, goals[0])
    if h_type == "min_md":
        #print("goals remaining:", goals)
        minDist = manhattanDist(start, goals[0])
        minGoal = goals[0]
        for i in range(len(goals)):
            if manhattanDist(start, goals[i]) < minDist:
                minDist = manhattanDist(start, goals[i])
                minGoal = goals[i]
        newGoals = goals.copy()
        newGoals.remove(minGoal)

        paths = 0
        otherDists = 0
        if tuple(newGoals) in minLens.keys():
            #print("DP")
            otherDists = minLens[tuple(newGoals)]
        else:
            # if unsearched -> do the mindist search
            newGoals2 = newGoals.copy()
            currGoal = minGoal
            while paths < len(goals) - 1:
                minD = manhattanDist(currGoal, newGoals[0])
                flag = False
                for g in newGoals:
                    if manhattanDist(currGoal, g) < minD:
                        minD = manhattanDist(currGoal, g)
                        currGoal = g
                        flag = True
                if flag is False:
                    newGoals.remove(newGoals[0])
                else:
                    newGoals.remove(currGoal)
                otherDists += minD
                paths += 1
            minLens[tuple(newGoals2)] = otherDists
        #print("minD:", otherDists)
        return minDist + otherDists / 3# * max(maze.getDimensions()[0], maze.getDimensions()[1])
    if h_type == "mst":
        minDist = manhattanDist(start, goals[0])
        minGoal = goals[0]
        for i in range(len(goals)):
            if manhattanDist(start, goals[i]) < minDist:
                minDist = manhattanDist(start, goals[i])
                minGoal = goals[i]
        newGoals = goals.copy()
        newGoals.remove(minGoal)
        #print("mst dist:", graph.mst(newGoals))
        return minDist + graph.mst(newGoals, minGoal)
    if h_type == "mst_fast":
        return 0
    return 0

def astarHelper(maze, start, goals, h_type):
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
    curr = (h(start, goals, h_type), 0, h(start, goals, h_type), start[0],
            start[1])
    heapq.heappush(heap, curr)

    wonSpot = None
    while len(heap) > 0:
        curr = heapq.heappop(heap)
        if (curr[3], curr[4]) in goals:
            wonSpot = curr
            break
        neighbors = maze.getNeighbors(curr[3], curr[4])
        for n in neighbors:
            newN = (h(n, goals, h_type) + curr[1] + 1, curr[1] + 1, h(n, goals, h_type), n[0], n[1])
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
    return astarHelper(maze, maze.getStart(), maze.getObjectives(), "md")
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

def findGoalOrder(start, goals):
    #do mst algorithm???
    #start from start, get lowest weight edge, add to tree if no cycle
    edges = [] #tuples: (distance, state1, state2)
    bestPath = [start]
    remainingVerts = goals
    #heapq.heappush(heap, newN)
    curr = start
    for g in goals:
        for v in remainingVerts:
            heapq.heappush(edges, (manhattanDist(curr, v), curr, v))

#also helper for part 3
def corner_helper(maze, start, goals, h_type, graph = None):
    # f = g(=path len) + mandist
    heap = []
    visited = {}
    tiebreaker = 1
    #         0  1  2  3  4    5           6           7        8
    # tuple: (f, g, h, x, y, tiebreaker, goals left, visited, currpath)
    curr = (h(start, goals, h_type, graph), 0, h(start, goals, h_type, graph), start[0],
            start[1], 0, goals, [])
    heapq.heappush(heap, curr)

    wonSpot = None
    while len(heap) > 0:
        curr = heapq.heappop(heap)
        '''
        print("q:", heap)
        print("visiting new node\n")
        print("f(x), g(x), h(x) =", curr[0], curr[1], curr[2])
        print("coords:", curr[3], curr[4])
        print("goals_left:", curr[6])
        print("visited:", visited)
        print("currpath:", curr[7], "\n")
        '''

        #if we hit a goal, rm from the list
        if (curr[3], curr[4]) in curr[6]:
            curr[6].remove((curr[3], curr[4]))
        #if there's no goals left, break + return path
        if len(curr[6]) == 0:
            wonSpot = curr
            #print("done!!!")
            break
        neighbors = maze.getNeighbors(curr[3], curr[4])
        for n in neighbors:
            curr6 = curr[6].copy()
            curr7 = curr[7].copy()
            tiebreaker += 1
            newN = (h(n, curr6, h_type, graph) + curr[1] + 1, curr[1] + 1, h(n, curr6, h_type, graph), n[0], n[1], tiebreaker, curr6, curr7)
            if (newN[3], newN[4]) not in visited or newN[6] not in visited[(newN[3], newN[4])][1]:
                # print("appending", n)
                #(newN[7])[(newN[3], newN[4])] = True #visited = True
                if (newN[3], newN[4]) not in visited:
                    visited[(newN[3], newN[4])] = ((newN[3], newN[4]), [])
                visited[(newN[3], newN[4])][1].append(newN[6])
                # add to path
                newN[7].append((curr[3], curr[4]))
                #print("adding n:", newN, type(newN), type(newN[0]))
                heapq.heappush(heap, newN)
    if wonSpot is None:
        print("no solution found")
        return []
    wonSpot[7].append((wonSpot[3], wonSpot[4]))
    #print(wonSpot[7])
    return wonSpot[7]


def astar_corner(maze):
    """
    Runs A star for part 2 of the assignment in the case where there are four corner objectives.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
        """
    # TODO: Write your code here

    return corner_helper(maze, maze.getStart(), maze.getObjectives(), "min_md")

def astar_multi(maze):
    """
    Runs A star for part 3 of the assignment in the case where there are
    multiple objectives.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """
    # TODO: Write your code here
    #setup graph stuff
    g = Graph(maze.getObjectives())
    #run old corner code w new mst h(x)
    return corner_helper(maze, maze.getStart(), maze.getObjectives(), "mst", g)


def fast(maze):
    """
    Runs suboptimal search algorithm for part 4.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """
    # TODO: Write your code here
    # setup graph stuff
    #g = Graph(maze.getObjectives())
    # run old corner code w new mst h(x)
    return []#corner_helper(maze, maze.getStart(), maze.getObjectives(), "mst_fast")
