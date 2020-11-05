import math
from queue import PriorityQueue
import matplotlib.pyplot as plt

class Graph:
    def __init__(self):
        #self.__vertexes = set()
        self.__vertexes = list()
        self.__adjencyList = dict()
        self.__vertexIDs = set()

    class Node:
        def __init__(self, id, x, y):
            self.__id = str(id)
            self.__x = float(x)
            self.__y = float(y)

        def getID(self):
            return self.__id
        def getX(self):
            return self.__x
        def getY(self):
            return self.__y
        def getCoords(self):
            return (self.__x, self.__y)

        def __eq__(self, other):
            return (self.__id == other.__id) or ((self.__x == other.__x) and ( self.__y == other.__y))
        def __ne__(self, other):
            return not self.__eq__(other)
        def __hash__(self):
            return hash((self.__id, self.__x, self.__y))

        def __str__(self):
            return "{}:({}, {})".format(self.__id, self.__x, self.__y)
        def __repr__(self):
            return "{}:({}, {})".format(self.__id, self.__x, self.__y)


    def addVertex(self, vertexData):
        node = self.Node(vertexData[0], vertexData[1], vertexData[2])

        #self.__vertexes.add( node )
        if node in self.__vertexes:
            print("Such vertex already exists")
            return
        self.__vertexes.append( node )
        self.__vertexIDs.add(vertexData[0])

    def addEdge(self, edgeData):
        if edgeData[0] not in self.__adjencyList:
            self.__adjencyList[ edgeData[0] ] = list()
        if edgeData[1] not in self.__adjencyList:
            self.__adjencyList[ edgeData[1] ] = list()

        if (edgeData[0] in self.__adjencyList[edgeData[1]]) or(edgeData[1] in self.__adjencyList[edgeData[0]]):
            print("Such edge already exists")
            return
        self.__adjencyList[edgeData[0]].append( edgeData[1] )
        self.__adjencyList[edgeData[1]].append( edgeData[0] )

    def getNodeFromID(self, id):
        for vertex in self.__vertexes:
            if vertex == self.Node(id, float("inf"), float("inf")):
                return vertex
        return None

    def loadDataFromFile(self, filename):
            fh = open(filename, "r")
            lines = fh.read().splitlines()

            #read vertices
            first_line_of_edges = -1
            for i in range(len(lines)):
                if lines[i] == "Edges:":
                    first_line_of_edges = i+1
                    break
                self.addVertex(lines[i].split(' '))
            #read edges
            for i in range(first_line_of_edges, len(lines)):
                self.addEdge( lines[i].split(' ') )





    def TSP_brute_force(self, start_id):
        start = self.getNodeFromID(start_id)
        path = list()
        best_tuple = self.recursive_brute(start, path, 0)
        return best_tuple


    def recursive_brute(self, working_vertex, path, distance_so_far):
        path = path.copy()
        working_vertex_ID = working_vertex.getID()
        path.append(working_vertex_ID)
        unvisited_vertexes = self.__vertexIDs - set(path)
        best_tuple = (float("inf"), list())
        if not unvisited_vertexes:#we are in the 'terminal' vertex, we just need to come back
            #print(path)
            starting_vertex_ID = path[0]
            vertex_to_make_cycle = self.getNodeFromID( starting_vertex_ID )
            if starting_vertex_ID in self.__adjencyList[working_vertex_ID]:#check if vertex is connected to ours
                distance_so_far += self.calcDistance(working_vertex, vertex_to_make_cycle)
                path.append(starting_vertex_ID)
                return (distance_so_far, path)
            return (float("inf"), list())
        else:
            IDs_of_neighbours = self.__adjencyList[working_vertex_ID]
            #unvisited_neighbours = set(IDs_of_neighbours) - set(path)
            #print(unvisited_neighbours)
            #if len(path) < len(self.__vertexIDs) and len(unvisited_vertexes) == 0:
            #    print("Graph does not contain a Hamiltion cicle")
            #    return None
            for neighbour_ID in IDs_of_neighbours:
                if neighbour_ID in unvisited_vertexes:
                    destVertex = self.getNodeFromID(neighbour_ID)
                    distance = distance_so_far
                    distance += self.calcDistance(working_vertex, destVertex)
                    tuple = self.recursive_brute(destVertex, path, distance)
                    if best_tuple[0] > tuple[0]:
                        best_tuple = tuple

        return best_tuple
    def TSP_closest_first(self, start_id):#using stack, so if we cannot continues we go back
        start_vertex = self.getNodeFromID(start_id)
        path = list()
        whole_distance = 0
        stack = list()
        stack.append((start_id, path, whole_distance))
        finished = False
        while (stack) and (not finished):
            tuple = stack.pop()
            current_id = tuple[0]
            path = tuple[1].copy()
            whole_distance = tuple[2]
            path.append(current_id)
            current_vertex = self.getNodeFromID(current_id)
            list_to_sort_neighbours = list()
            for neighbour_id in self.__adjencyList[current_id]:
                if neighbour_id in path:
                    continue
                neighbour_vertex = self.getNodeFromID(neighbour_id)
                new_distance = self.calcDistance(current_vertex, neighbour_vertex)
                list_to_sort_neighbours.append( (new_distance, neighbour_id) )
            list_to_sort_neighbours.sort(reverse = True)
            for neighbour_data in list_to_sort_neighbours:
                stack.append( (neighbour_data[1], path, neighbour_data[0]+whole_distance) )
            if len(path) == len(self.__vertexIDs):
                if start_id in self.__adjencyList[current_id]:
                    path.append(start_id)
                    whole_distance += self.calcDistance(current_vertex, start_vertex)
                    return (whole_distance, path)
    def TSP_A_star(self, start_id):
        start = self.getNodeFromID(start_id)
        path = list()

    def calcDistance(self, src, dest):
        #   sqrt( (x1-x2)^2 + (y1-y2)^2 )
        return math.sqrt( (src.getX() - dest.getX())**2 + (src.getY()-dest.getY())**2)


    def drawGraph(self):
        x_coords = list()
        y_coords = list()
        for vertex in self.__vertexes:
            x_coords.append(vertex.getX())
            y_coords.append(vertex.getY())
        plt.scatter(x_coords, y_coords)
        plt.show()

    def drawPath(self, paths):
        x_coords = list()
        y_coords = list()
        for vertex in self.__vertexes:
            x_coords.append(vertex.getX())
            y_coords.append(vertex.getY())
        for path_data in paths:
            path_x_coords = list()
            path_y_coords = list()
            for ID in path_data[0]:
                node = self.getNodeFromID(ID)
                path_x_coords.append(node.getX())
                path_y_coords.append(node.getY())
            plt.plot(path_x_coords, path_y_coords, label = path_data[1])
            plt.legend()
        plt.scatter(x_coords, y_coords)
        plt.show()

        
    def printGraph(self):
        for node in self.__vertexes:
            print(node)
        keys = self.__adjencyList.keys()
        for key in keys:
            print(key + ": ", end='')
            for dest in self.__adjencyList[key]:
                print(dest, end=' ')
            print()



def main():
    graph = Graph()
    graph.loadDataFromFile("data.txt")
    dist_path_bruteForce = graph.TSP_brute_force("A")
    dist_path_closestFirst = graph.TSP_closest_first("A")
    print(dist_path_bruteForce)
    print(dist_path_closestFirst)
    paths = ((dist_path_bruteForce[1], "brute force"), (dist_path_closestFirst[1], "closest first"))
    graph.drawPath(paths)



if __name__ == "__main__":
    main()
