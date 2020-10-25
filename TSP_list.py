import math

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
            indexes_of_unvisited_vertexes.remove(index_of_shortest)
            current = index_of_shortest
        path.append(start)#add starting node since we need to make a cycle
        #need to count the total distance
        return path

    def TSP_A_star(self):
        pass
    def TSP_brute_force(self):
        pass

    def calcDistance(self, src, dest):
        #   sqrt( (x1-x2)^2 + (y1-y2)^2 )
        return math.sqrt( (src.getX() - dest.getX())**2 + (src.getY()-dest.getY())**2)

def main():
    graph = Graph()
    graph.loadDataFromFile("data.txt")
    print(graph.TSP_closest_first())


if __name__ == "__main__":
    main()
