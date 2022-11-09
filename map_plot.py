# Author: Jordan Kirkbride
# Date: 06/02/2021
# Lab 4

from cs1lib import *
from load_graph import *
from bfs import *

# Constants for design
WIDTH = 1012
HEIGHT = 811
BLUE = [0, 0, 1]
RED = [1, 0, 0]

# Loads files
img = load_image("dartmouth_map.png")
v_obj_dic = load_graph("dartmouth_graph.txt")

starting_vertex = None
goal_vertex = None


# Draws the map and its vertices with edges
def draw():
    clear()
    draw_image(img, 0, 0)

    # Draws each vertex and edge
    for key in v_obj_dic:   # Draws each blue vertex
        if v_obj_dic[key] != starting_vertex:
            v_obj_dic[key].draw_vertex(BLUE[0], BLUE[1], BLUE[2])

        # Draws a red starting vertex
        if starting_vertex != None:
            starting_vertex.draw_vertex(RED[0], RED[1], RED[2])

        # Draws a red goal vertex
        if goal_vertex != None:
            goal_vertex.draw_vertex(RED[0], RED[1], RED[2])

        # Draws each edge blue
        v_obj_dic[key].draw_edge(BLUE[0], BLUE[1], BLUE[2])

        # Shortest path is determined using BFS
        path = bfs(starting_vertex, goal_vertex)

        # Draws the shortest path from start to goal in red
        for i in range(len(path)):
            if (i+1) < len(path):
                set_stroke_color(RED[0], RED[1], RED[2])
                draw_line(int(path[i].x), int(path[i].y), int(path[i+1].x), int(path[i+1].y))
                path[i].draw_vertex(RED[0], RED[1], RED[2])

# Determines if the mouse is pressed over a vertex
def my_mouse_press(x, y):
    for key in v_obj_dic:
        # Determines if the mouse is pressed while inside of a vertex
        if (int(v_obj_dic[key].x)-8) <= x <= (int(v_obj_dic[key].x)+8) and (int(v_obj_dic[key].y)-8) <= y <= (int(v_obj_dic[key].y)+8):
            v_obj_dic[key].mouse_pressed = True

def my_mouse_release(x, y):
    global starting_vertex
    for key in v_obj_dic:
        # Determines if the mouse is released while inside of a vertex AND is the same vertex the mouse pressed on
        # If so, the starting vertex has been picked and is assigned
        if (int(v_obj_dic[key].x)-8) <= x <= (int(v_obj_dic[key].x)+8) and (int(v_obj_dic[key].y)-8) <= y <= (int(v_obj_dic[key].y)+8) and v_obj_dic[key].mouse_pressed == True:
            v_obj_dic[key].mouse_released = True
            starting_vertex = v_obj_dic[key]

# Determines if the mouse is over a vertex, if so and if there is already a starting vertex then that is the goal vertex
def my_mouse_move(x, y):
    global goal_vertex
    if starting_vertex != None:
        for key in v_obj_dic:
            # Determines if mouse is located on a vertex.
            # If so, assigns that as the goal vertex only if a starting vertex has already been selected
            if (int(v_obj_dic[key].x) - 8) <= x <= (int(v_obj_dic[key].x) + 8) and (int(v_obj_dic[key].y) - 8) <= y <= (int(v_obj_dic[key].y) + 8):
                goal_vertex = v_obj_dic[key]


start_graphics(draw, width=WIDTH, height=HEIGHT, mouse_press=my_mouse_press, mouse_release=my_mouse_release,
               mouse_move=my_mouse_move)