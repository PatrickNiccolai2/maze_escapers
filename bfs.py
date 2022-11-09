# Author: Jordan Kirkbride
# Date: 06/02/2021
# Lab 4
from collections import deque

# Using breadth first search, determines the shortest path from start to goal
def bfs(start, goal):
    bfs_dic = {}
    path = []
    goal_reached = False
    bfs_dic[start] = None

    frontier = deque()
    frontier.append(start)

    # In waves, the goal is searched for
    while len(frontier) > 0:
        v = frontier.popleft()
        # Each adjacent vertex of start is searched
        if v != None:
            for each in v.adjacent_vertices:
                # Determines if a vertex has already been searched
                if each in bfs_dic:
                    pass
                else:
                    frontier.append(each)
                    bfs_dic[each] = v
        # Breaks if goal has been found
        if v == goal:
            goal_reached = True
            break
    # If the goal has been found, the path list is created and returned
    if goal_reached:
        curr = goal
        while curr != None:
            path.append(curr)
            curr = bfs_dic[curr]

    return path
