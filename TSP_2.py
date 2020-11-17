import math
import time
from queue import PriorityQueue

import matplotlib.pyplot as plt


class Graph:
    def __init__(self):
        # self.__vertexes = set()
        self.__vertexes = list()
        self.__adjacencyList = dict()

        self.__vertexIDs = set()  # to make algorithms a little bit faster

        self.__number_of_vertexes = 0 # to compare algorithms
        self.__number_of_edges = 0

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
            return self.__x, self.__y

        def __eq__(self, other):
            return (self.__id == other.__id) or ((self.__x == other.__x) and (self.__y == other.__y))

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

        # self.__vertexes.add( node )
        if node in self.__vertexes:
            print("Such vertex already exists")
            return
        self.__vertexes.append(node)
        self.__vertexIDs.add(vertexData[0])
        self.__number_of_vertexes += 1

    def addEdge(self, edgeData):
        if edgeData[0] not in self.__adjacencyList:
            self.__adjacencyList[edgeData[0]] = list()
        if edgeData[1] not in self.__adjacencyList:
            self.__adjacencyList[edgeData[1]] = list()

        if edgeData[0] == edgeData[1]:
            print("No edges to itself allowed")
            return
        if (edgeData[0] in self.__adjacencyList[edgeData[1]]) or (edgeData[1] in self.__adjacencyList[edgeData[0]]):
            print("Such edge already exists")
            return
        self.__adjacencyList[edgeData[0]].append(edgeData[1])
        self.__adjacencyList[edgeData[1]].append(edgeData[0])

        self.__number_of_edges += 1

    def getNodeFromID(self, id):
        for node in self.__vertexes:
            if node == self.Node(id, float("inf"), float("inf")):
                return node
        return None

    def loadDataFromFile(self, filename):
        fh = open(filename, "r")
        lines = fh.read().splitlines()

        # read vertices
        first_line_of_edges = -1
        for i in range(len(lines)):
            if lines[i] == "Edges:":
                first_line_of_edges = i + 1
                break
            self.addVertex(lines[i].split(' '))
        # read edges
        for i in range(first_line_of_edges, len(lines)):
            self.addEdge(lines[i].split(' '))

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
        if not unvisited_vertexes:  # we are in the 'terminal' vertex, we just need to come back
            # print(path)
            starting_vertex_ID = path[0]
            vertex_to_make_cycle = self.getNodeFromID(starting_vertex_ID)
            if starting_vertex_ID in self.__adjacencyList[working_vertex_ID]:  # check if vertex is connected to ours
                distance_so_far += self.calcDistance(working_vertex, vertex_to_make_cycle)
                path.append(starting_vertex_ID)
                return distance_so_far, path
            return float("inf"), list()
        else:
            IDs_of_neighbours = self.__adjacencyList[working_vertex_ID]
            # unvisited_neighbours = set(IDs_of_neighbours) - set(path)
            # print(unvisited_neighbours)
            # if len(path) < len(self.__vertexIDs) and len(unvisited_vertexes) == 0:
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

    def TSP_closest_first(self, start_id):  # using stack, so if we cannot continue we go back
        start_vertex = self.getNodeFromID(start_id)
        path = list()
        whole_distance = 0
        stack = list()
        stack.append((start_id, path, whole_distance))
        finished = False
        while stack and (not finished):
            tuple = stack.pop()
            current_id = tuple[0]
            path = tuple[1].copy()
            whole_distance = tuple[2]
            path.append(current_id)
            current_vertex = self.getNodeFromID(current_id)
            list_to_sort_neighbours = list()
            for neighbour_id in self.__adjacencyList[current_id]:
                if neighbour_id in path:
                    continue
                neighbour_vertex = self.getNodeFromID(neighbour_id)
                new_distance = self.calcDistance(current_vertex, neighbour_vertex)
                list_to_sort_neighbours.append((new_distance, neighbour_id))
            list_to_sort_neighbours.sort(reverse=True)
            for neighbour_data in list_to_sort_neighbours:
                stack.append((neighbour_data[1], path, neighbour_data[0] + whole_distance))
            if len(path) == len(self.__vertexIDs):
                if start_id in self.__adjacencyList[current_id]:
                    path.append(start_id)
                    whole_distance += self.calcDistance(current_vertex, start_vertex)
                    return whole_distance, path

    def TSP_A_star(self, start_id, param):
        if param != 1 and param != 2:
            print("Invalid parameter")
            return
        start_vertex = self.getNodeFromID(start_id)
        path = list()

        # number of terminal points depends on graph density
        number_of_terminal_points = len(self.__adjacencyList[start_id]) * (self.__number_of_edges)
        best_tuple = (None, float("inf"))

        collection = self.getHeuristicCollection(param)

        pQueue = PriorityQueue()
        path.append(start_id)
        distance_so_far = 0
        # pQueue contains tuple of:
        # priority ; path_so_far, heuristic_collection ; sum_of_distance_so_far
        if type(collection) is list:
            pQueue.put((sum(collection), path, collection, distance_so_far))
        elif type(collection) is dict:
            pQueue.put((sum(collection.values()), path, collection, distance_so_far))

        while not pQueue.empty():
            tuple = pQueue.get()
            path = tuple[1]
            collection_copy = tuple[2]
            distance_so_far = tuple[3]

            current_vertex_id = path[-1]
            current_vertex = self.getNodeFromID(current_vertex_id)

            unvisited_vertexes = self.__vertexIDs - set(path)
            if unvisited_vertexes:
                for neighbour_id in self.__adjacencyList[current_vertex_id]:

                    if neighbour_id in unvisited_vertexes:
                        tmp_collection_copy = collection_copy.copy()
                        tmp_path = path.copy()
                        tmp_path.append(neighbour_id)
                        neighbour_vertex = self.getNodeFromID(neighbour_id)
                        new_distance = self.calcDistance(current_vertex, neighbour_vertex)
                        heuristic_value = self.getHeuristicValueAndAmendCollectionState(tmp_collection_copy, neighbour_id, new_distance)
                        new_distance = distance_so_far + new_distance
                        priority = heuristic_value + new_distance
                        new_tuple = (priority, tmp_path, tmp_collection_copy, new_distance)
                        pQueue.put(new_tuple)
            else:
                if start_id in self.__adjacencyList[current_vertex_id]:
                    number_of_terminal_points -= 1

                    new_distance = self.calcDistance(current_vertex, start_vertex)
                    distance_so_far += new_distance
                    path.append(start_id)

                    if distance_so_far < best_tuple[1]:
                        best_tuple = (path, distance_so_far)
                    if number_of_terminal_points == 0:
                        break
                else:  # we cannot come back, continue looking
                    continue
        return best_tuple[1], best_tuple[0]

    def getHeuristicCollection(self, param):
        if param == 1:  # just shortest edges in graph
            edges = list()
            for vertex_id in self.__vertexIDs:
                vertex = self.getNodeFromID(vertex_id)
                for neighbour_id in self.__adjacencyList[vertex_id]:
                    neighbour = self.getNodeFromID(neighbour_id)
                    distance = self.calcDistance(vertex, neighbour)
                    if (distance, neighbour_id, vertex_id) in edges:
                        continue
                    edges.append((distance, vertex_id, neighbour_id))
            edges.sort()
            shortest_edges = edges[:len(self.__vertexIDs)]
            shortest_distances_left = list()
            for edge in shortest_edges:
                shortest_distances_left.append(edge[0])
            del shortest_edges
            return shortest_distances_left
        elif param == 2:  # shortest edge of each vertex
            shortest_distances_dict = dict()
            for vertex_id in self.__vertexIDs:
                vertex_node = self.getNodeFromID(vertex_id)
                min = float("inf")
                for neighbour_id in self.__adjacencyList[vertex_id]:
                    neighbour_node = self.getNodeFromID(neighbour_id)
                    distance = self.calcDistance(vertex_node, neighbour_node)
                    if min > distance:
                        min = distance
                    shortest_distances_dict[vertex_id] = min
            return shortest_distances_dict
        return None

    def getHeuristicValueAndAmendCollectionState(self, collection, vertex_id, distance):
        if type(collection) is list:
            if distance in collection:
                collection.remove(distance)
            elif collection:
                del collection[-1]
            return sum(collection)
        elif type(collection) is dict:
            if vertex_id in collection:
                del collection[vertex_id]
            return sum(collection.values())
        return None

    def calcDistance(self, src, dest):
        #   sqrt( (x1-x2)^2 + (y1-y2)^2 )
        return math.sqrt((src.getX() - dest.getX()) ** 2 + (src.getY() - dest.getY()) ** 2)

    def drawGraph(self):
        x_coords = list()
        y_coords = list()
        path_x_coords = list()
        path_y_coords = list()
        plt.figure("Graph")
        for vertex in self.__vertexes:
            x_coords.append(vertex.getX())
            y_coords.append(vertex.getY())
            for ID in self.__adjacencyList[vertex.getID()]:
                node = self.getNodeFromID(ID)
                path_x_coords.append(vertex.getX())
                path_y_coords.append(vertex.getY())
                path_x_coords.append(node.getX())
                path_y_coords.append(node.getY())
                plt.plot(path_x_coords, path_y_coords, color='Blue')
                path_y_coords.clear()
                path_x_coords.clear()
        
        plt.scatter(x_coords, y_coords, color='Red')       
        #plt.show()

    def drawPaths(self, paths):
        x_coords = list()
        y_coords = list()
        for vertex in self.__vertexes:
            x_coords.append(vertex.getX())
            y_coords.append(vertex.getY())
        for path_data in paths:
            plt.figure(path_data[1])
            path_x_coords = list()
            path_y_coords = list()
            for ID in path_data[0]:
                node = self.getNodeFromID(ID)
                path_x_coords.append(node.getX())
                path_y_coords.append(node.getY())
            plt.plot(path_x_coords, path_y_coords, label=path_data[1], color='k')
            for i in range(len(path_x_coords)):
                if i != 0:
                    plt.arrow(path_x_coords[i-1], path_y_coords[i-1], path_x_coords[i]-path_x_coords[i-1], path_y_coords[i]-path_y_coords[i-1],length_includes_head=True, head_width=0.5, head_length=0.5, fc='k', ec='k')
            plt.legend()
            plt.scatter(x_coords, y_coords)

        plt.scatter(x_coords, y_coords)
        plt.show()

    def printGraph(self):
        for node in self.__vertexes:
            print(node)
        keys = self.__adjacencyList.keys()
        for key in keys:
            print(key + ": ", end='')
            for dest in self.__adjacencyList[key]:
                print(dest, end=' ')
            print()

    def printGraphData(self):
        density_percentage = (2*self.__number_of_edges)/((self.__number_of_vertexes-1)*self.__number_of_vertexes) * 100
        print("Number of vertexes: {}\tGraph density: {}%".format(self.__number_of_vertexes, density_percentage))


def main():
    graph = Graph()
    graph.loadDataFromFile("data_graphs/data.txt")
    first_vertex = "A"
    brute_time = time.time()
    dist_path_bruteForce = graph.TSP_brute_force(first_vertex)
    print("Brute force time:   %s seconds" % (time.time() - brute_time))

    closest_first_time = time.time()
    dist_path_closestFirst = graph.TSP_closest_first(first_vertex)
    print("Closest first time: %s seconds" % (time.time() - closest_first_time))

    A_star_time = time.time()
    dist_path_A_star = graph.TSP_A_star(first_vertex, 1)
    print("A* time:            %s seconds" % (time.time() - A_star_time))

    A_star_time2 = time.time()
    dist_path_A_star2 = graph.TSP_A_star(first_vertex, 2)
    print("A*2 time:           %s seconds" % (time.time() - A_star_time2))

    print("BruteForce  : " + str(dist_path_bruteForce))
    print("ClosestFirst: " + str(dist_path_closestFirst))
    print("A_star      : " + str(dist_path_A_star))
    print("A_star2     : " + str(dist_path_A_star2))

    graph.printGraphData()

    paths = list()
    paths.append((dist_path_bruteForce[1], "brute force"))
    paths.append((dist_path_closestFirst[1], "closest first"))
    paths.append((dist_path_A_star[1], "A_star"))
    paths.append((dist_path_A_star2[1], "A_star2"))

    graph.drawGraph()
    graph.drawPaths(paths)


if __name__ == "__main__":
    main()
