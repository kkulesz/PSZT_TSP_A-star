import math
from queue import PriorityQueue
import numpy as np
import matplotlib.pyplot as plt
import itertools

import gc


class Graph:
    class Node:
        def __init__(self, x, y):
            self.__x = int(x)
            self.__y = int(y)

        def getX(self):
            return self.__x
        def getY(self):
            return self.__y
        def getCoords(self):
            return (self.__x, self.__y)
        #te 3 ponizsze funkcje to okazaly sie niepotrzebne,
        #początkowo chciałem przechowywać wierzchołki w zbiorze zamiast w liście
        #ale tak bedzie trudniej i chyba żadnej optymalizacji z tego powodu
        def __eq__(self, other):
            return (self.__x == other.__x) and ( self.__y == other.__y)
        def __ne__(self, other):
            return not self.__eq__(other)
        def __hash__(self):
            return hash((self.__x, self.__y))
        def __str__(self):
            return "({}, {})".format(self.__x, self.__y)
        def __repr__(self):
            return "({}, {})".format(self.__x, self.__y)

    def __init__(self):
        self.__vertexes = list()

    def addVertex(self, x, y):
        node = self.Node(x,y)
        if node in self.__vertexes:
            return
        self.__vertexes.append( node )

    def addVertex(self, coords):
        node = self.Node(coords[0], coords[1])
        if node in self.__vertexes:
            return
        self.__vertexes.append( node )

    def loadDataFromFile(self, fileName):
        fh = open(fileName, "r")
        lines = fh.read().splitlines()

        for line in lines:
            self.addVertex( line.split(' ') )

        fh.close()

    def TSP_closest_first(self):
        if len(self.__vertexes) == 0:
            return

        indexes_of_unvisited_vertexes = [i for i in range(1, len(self.__vertexes) )]
        start = 0# starting with first node of the list since it makes no difference where we are starting
        path = list()
        distance = 0
        path.append(start)
        current = start
        while indexes_of_unvisited_vertexes:
            index_of_shortest = indexes_of_unvisited_vertexes[0]
            shortest_distance = float("inf")
            for i in indexes_of_unvisited_vertexes:
                dist = self.calcDistance( self.__vertexes[current], self.__vertexes[i] )
                if shortest_distance > dist:
                    index_of_shortest = i
                    shortest_distance = dist

            #add vertex with the shortest distance to the path,
            #remove him from unvisited_vertexes, because we do not want to visit the same vertex again.
            #and make him the new 'working point'
            path.append(index_of_shortest)
            distance += shortest_distance
            indexes_of_unvisited_vertexes.remove(index_of_shortest)
            current = index_of_shortest
        path.append(start)#add starting node since we need to make a cycle
        distance += self.calcDistance(self.__vertexes[index_of_shortest], self.__vertexes[start])
        return (path, distance)

    #we prioritize by
    def TSP_A_star(self):
        shortest_lens_of_edges = list()
        for i in range( len(self.__vertexes) ):
            for j in range( i+1, len(self.__vertexes) ):
                shortest_lens_of_edges.append( self.calcDistance(self.__vertexes[i], self.__vertexes[j]) )
        shortest_lens_of_edges.sort()
        shortest_lens_of_edges = shortest_lens_of_edges[:len(self.__vertexes)+1] # trim to the number of edges in hammilton cycle
        """
            #nonsense since for the same 'deepness level' nodes we will be getting the same value,
            #meaning we can as well add 0 everywhere, which will give as 'closest first' alghoritm
        sums_of_lens = [0]
        current_sum = 0
        for next_len in shortest_lens_of_edges:
            current_sum += next_len
            sums_of_lens.append(current_sum)

        #sums_of_lens - for given index will return estimated distance left
        # i.e. if 3 towns left, then the estimated distance will be sums_of_lens[3]
        #del shortest_lens_of_edges # no longer needed
        """

        start = 0
        init_list = [start]
        best_path_so_far = list()
        best_result = float("inf")
        pQueue = PriorityQueue()
        init_priority = sum(shortest_lens_of_edges)#self.heuristic_func(start, list(start), shortest_lens_of_edges)
        pQueue.put((init_priority, init_list, shortest_lens_of_edges, 0))
        allowable_terminal_points = 500 # after reaching this number, alghoritm will return best result found to this point
        while allowable_terminal_points:
            tuple = pQueue.get()
            path = tuple[1]
            shortest_distances_left = tuple[2]
            distance_so_far = tuple[3]
            current = path[-1]#
            unvisited_vertexes_indexes = list( set([i for i in range(len(self.__vertexes))]) - set(path) )
            #print(str(current) + " "+ str(distance_so_far) + " " + str(path))
            if not unvisited_vertexes_indexes:
                #print("##################################################################")
                allowable_terminal_points-=1
                distance_to_come_back = self.calcDistance( self.__vertexes[current], self.__vertexes[start])
                distance_so_far += distance_to_come_back
                path.append(start)
                if best_result > distance_so_far:
                    best_result = distance_so_far
                    best_path_so_far = path.copy()
                continue
            for i in range( min(4, len(unvisited_vertexes_indexes)) ):#ONLY FIVE in order to not achive complexity of (n!)
                tmp_path = path.copy()
                neighbour_index = unvisited_vertexes_indexes[i]
                tmp_path.append(neighbour_index)
                new_distance = self.calcDistance( self.__vertexes[current], self.__vertexes[neighbour_index])
                new_shortest_distances_left = shortest_distances_left.copy()
                if new_distance in new_shortest_distances_left:
                    new_shortest_distances_left.remove(new_distance)
                else:
                    del new_shortest_distances_left[-1]
                distance = distance_so_far+new_distance
                priority = sum(new_shortest_distances_left)
                priority += distance
                #print(priority)
                pQueue.put( (priority, tmp_path, new_shortest_distances_left, distance))
        return (best_path_so_far, best_result)

    def TSP_brute_force(self):
        pass

    def calcDistance(self, src, dest):
        #   sqrt( (x1-x2)^2 + (y1-y2)^2 )
        return math.sqrt( (src.getX() - dest.getX())**2 + (src.getY()-dest.getY())**2)

    def drawGraph(self):
        x_coords = list()
        y_coords = list()
        for vertex in self.__vertexes:
            x_coords.append(vertex.getX())
            y_coords.append(vertex.getY())
        #plt.plot(x_coords, y_coords, linestyle='')
        plt.scatter(x_coords, y_coords)
        plt.show()
def main():
    graph = Graph()
    graph.loadDataFromFile("data.txt")
    graph.drawGraph()
    print(graph.TSP_closest_first())
    print(graph.TSP_A_star())

if __name__ == "__main__":
    main()
