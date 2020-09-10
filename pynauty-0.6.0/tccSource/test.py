from pynauty import *
import json

def getIsIsomorphic(g, h):
    return isomorphic(g, h)


def getGraph(data):
    g = Graph(data["numberOfVertex"])
    graph = data["graph"]
    for v in graph:
        g.connect_vertex(int(v), graph[v])
    return g

def main():
    with open("graphs.json") as jsonFile:
        data = json.load(jsonFile)
    g = getGraph(data[0])
    h = getGraph(data[1])
    print(getIsIsomorphic(g, h))

main()
