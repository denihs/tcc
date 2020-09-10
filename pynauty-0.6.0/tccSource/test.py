from pynauty import *
import json
from GraphClass import GraphExt

def getIsIsomorphic(g, h):
    return isomorphic(g, h)


def getGraph(data):
    g = GraphExt(data["numberOfVertex"])
    graph = data["graph"]
    for v in graph:
        g.connect_vertex(int(v), graph[v])
    return g

def main():
    with open("graphs.json") as jsonFile:
        data = json.load(jsonFile)
    g = getGraph(data[0])
    h = getGraph(data[1])

    for i in range(g.vertexAmount):
        print("{} - {}".format(i, g.getVertexAdjacency(i)))

    print(getIsIsomorphic(g, h))

main()
