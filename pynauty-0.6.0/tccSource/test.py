from pynauty import isomorphic
import json
from GraphClass import GraphExt
from copy import deepcopy
from itertools import permutations
import igraph

GRAPHS = []
ACCEPTED_GRAPHS = []
GRAPHS_COUNT = 0
ISOMORPHIC_GRAPH_COUNT = 0


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
    g = GraphExt(newGraphSize, bindDegree=graph1.getVertexDegree(graph1.bindVertex))
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

    # isso ajudará quando for necessário permutar um dos grafos na colagem
    g.setInternalGraphEnd(len(map1.keys()) - 1)
    g.setInternalGraphVertexAmount(len(map1.keys()))
    g.setExternalGraphVertexAmount(len(map2.keys()))

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
                g.setConnections(key, p)
                g.connect_vertex(key, [p] + g.adjacency_dict[key])

    return g


def twistEdges(graph):
    global GRAPHS
    global GRAPHS_COUNT
    global ACCEPTED_GRAPHS
    global ISOMORPHIC_GRAPH_COUNT

    connections = graph.connections
    connectionsLen = len(connections.keys())

    if connectionsLen < 4:
        return

    keys = list(connections.keys())
    perm = list(permutations(connections.values()))

    ACCEPTED_GRAPHS.append(graph)

    for vertices in perm:
        g = deepcopy(graph)
        print("{} {}".format(keys, vertices))
        for k, v in zip(keys, vertices):
            adj = g.getAllVertexAdjacency()
            g.connect_vertex(
                k,
                list(
                    set(
                        filterList(
                            adj[k],
                            connections[k]
                        ) +
                        [v]
                    )
                )
            )

        GRAPHS_COUNT += 1

        for acceptedGraph in ACCEPTED_GRAPHS:
            if getIsIsomorphic(g, acceptedGraph):
                ISOMORPHIC_GRAPH_COUNT += 1
                break
        else:
            ACCEPTED_GRAPHS.append(g)


def drawGraph(g, name=None, gFormat="circular"):
    gCopy = deepcopy(g)
    allAdjacency = gCopy.getAllVertexAdjacency()
    graph = igraph.Graph()
    graph.add_vertices(gCopy.vertexAmount)
    vertices = []
    for vertex, adjacency in allAdjacency.items():
        for v in adjacency:
            if (v, vertex) not in vertices:
                vertices.append((vertex, v))
    graph.add_edges(vertices)
    graph.vs["label"] = list(allAdjacency.keys())
    layout = graph.layout(gFormat)
    igraph.plot(graph, layout=layout, inline=True).save('./img/{}.png'.format(name))


def main():
    global GRAPHS
    global ACCEPTED_GRAPHS
    global GRAPHS_COUNT
    global ISOMORPHIC_GRAPH_COUNT

    with open("grafosColagemRoda5v.json") as jsonFile:
        data = json.load(jsonFile)

    g = getGraph(data[0])
    h = getGraph(data[1])

    drawGraph(g, "graph-init-1", gFormat="kk")
    drawGraph(h, "graph-init-2", gFormat="kk")

    bindedGraph = bindGraph(g, h)

    twistEdges(bindedGraph)
    count = 1
    for g in ACCEPTED_GRAPHS:
        print("-=-==- GRAFO {} -=-==- ".format(count))
        print(g)
        drawGraph(g, "graph-{}".format(count))
        count += 1
    print("TOTAL GRAFOS GERADOS: {}".format(GRAPHS_COUNT))
    print("TOTAL GRAFOS ISOMORFOS: {}".format(ISOMORPHIC_GRAPH_COUNT))


main()
