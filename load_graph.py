# Author: Jordan Kirkbride
# Date: 06/02/2021
# Lab 4
from vertex import Vertex

def load_graph(name):
    vertex_objects = {}

    in_file = open(f"{name}", "r")

    # For each line in the file, the data is split up and added to the appropriate instance variables of
    # its specific vertex object
    for each in in_file:
        # name is stored
        s1 = each.strip()
        s2 = s1.split(";")
        name = s2[0]
        
        # list of adj vertices are stored
        list_of_adj = s2[1].split(",")

        # coordinates of the vertex are stored
        coordinates = s2[2].split(",")
        x = coordinates[0].strip()
        y = coordinates[1].strip()
        
        # vertex object is created from previously determined variables and is then stored in a dictionary
        v = Vertex(name, x, y, list_of_adj)
        vertex_objects[f"{name}"] = v

    in_file.close()

    # Creates a list of adjacent vertex objects for each item in the dictionary
    for each in vertex_objects:
        list_of_vertices = []
        for v in vertex_objects[each].adjacent_vertices:
            s = v.strip()
            list_of_vertices.append(vertex_objects[s])

        vertex_objects[f"{each}"].adjacent_vertices = list_of_vertices

    return vertex_objects
