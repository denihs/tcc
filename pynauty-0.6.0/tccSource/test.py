from pynauty import *
import json
from GraphClass import GraphExt


def getIsIsomorphic(g, h):
    return isomorphic(g, h)


def getGraph(data):
    g = GraphExt(data["numberOfVertex"], data["bindVertex"])
    graph = data["graph"]
    for v in graph:
        g.connect_vertex(int(v), graph[v])
    return g


def translateMap(graphMap, adjacency):
    listGraph = []
    for vertex in adjacency:
        listGraph.append(list(graphMap.keys())[list(graphMap.values()).index(vertex)])
    return listGraph


def filterList(listValues, value):
    return list(x for x in listValues if x != value)


def bindGraph(g1, g2):
    if g1.vertexAmount < g2.vertexAmount:
        graph1 = g1
        graph2 = g2
    else:
        graph1 = g2
        graph2 = g1

    if graph1.getVertexDegree(graph1.bindVertex) != graph2.getVertexDegree(graph2.bindVertex):
        raise Exception("bindGraph - grau dos vétices da colagem devem ser o mesmo")

    graphAdjacency1 = graph1.getAllVertexAdjacency()
    graphAdjacency2 = graph2.getAllVertexAdjacency()
    newGraphSize = graph1.vertexAmount + graph2.vertexAmount - 2

    # mapeando ambos os grafos para um grafo apenas
    map1 = {}
    map2 = {}
    keyList1 = filterList(graphAdjacency1.keys(), graph1.bindVertex)
    keyList2 = filterList(graphAdjacency2.keys(), graph2.bindVertex)
    for i in range(newGraphSize):
        if i < graph1.vertexAmount - 1:
            elem = keyList1.pop()
            map1[i] = elem
        else:
            elem = keyList2.pop()
            map2[i] = elem

    # montando o grafo
    g = GraphExt(newGraphSize)
    for i in range(newGraphSize):
        if i < graph1.vertexAmount - 1:
            g.connect_vertex(
                i,
                translateMap(
                    map1,
                    filterList(graphAdjacency1[map1[i]], graph1.bindVertex)
                )
            )
        else:
            g.connect_vertex(
                i,
                translateMap(
                    map2,
                    filterList(graphAdjacency2[map2[i]], graph2.bindVertex)
                )
            )

    # conectando as partes dos grafos
    # descobre quem está com grau faltando no primeiro grafo
    # e adiciona uma aresta para todos os vértices dele
    gDegrees = g.getVertexDegree()
    graph1Degrees = graph1.getVertexDegree()
    diffs = {}
    for key, degree in gDegrees.items():
        if key < graph1.vertexAmount - 1:
            if degree < graph1Degrees[map1[key]]:
                diffs[key] = graph1Degrees[map1[key]] - degree

    mapKeys2 = list(map2.keys())
    for key, diff in diffs.items():
        if key < graph1.vertexAmount - 1:
            p = mapKeys2.pop()
            for i in range(diff):
                g.connect_vertex(key, [p] + g.adjacency_dict[key])

    return g


def main():
    with open("grafosColagem4v.json") as jsonFile:
        data = json.load(jsonFile)

    g = getGraph(data[0])
    h = getGraph(data[1])

    bindedGraph = bindGraph(g, h)
    print(bindedGraph)

main()
