# Author: Jordan Kirkbride
# Date: 06/02/2021
# Lab 4
from cs1lib import *

WIDTH = 3
RADIUS = 8

# Vertex class used to store each vertex on map
class Vertex:
    def __init__(self, name, x, y, plist):
        self.name = name
        self.x = x
        self.y = y
        self.adjacent_vertices = plist
        self.mouse_pressed = False
        self.mouse_released = False

    def __str__(self):
        s = ""
        for each in self.adjacent_vertices:
            s += f"{each.name}, "
        s = s.strip()
        s = s.strip(",")

        return f"{self.name}; Location: {self.x}, {self.y}; Adjacent Vertices: {s}"

    # Draws the vertex
    def draw_vertex(self, r, g, b):
        set_stroke_color(r, g, b)
        set_fill_color(r, g, b)
        draw_circle(int(self.x), int(self.y), RADIUS)

    # Draws each edge of the vertex connecting to its adjacent vertices
    def draw_edge(self, r, g, b):
        set_stroke_width(WIDTH)
        set_stroke_color(r, g, b)
        for each in self.adjacent_vertices:
            draw_line(int(self.x), int(self.y), int(each.x), int(each.y))
