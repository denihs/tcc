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

K4 = {
    "numberOfVertex": 4,
    "bindVertex": 0,
    "graph": {
        "0": [1, 2, 3],
        "1": [2, 3],
        "2": [3],
    }
}

K33 = {
    "numberOfVertex": 6,
    "bindVertex": 0,
    "graph": {
        "0": [3, 4, 5],
        "1": [3, 4, 5],
        "2": [3, 4, 5],
    }
}

S8 = {
    "numberOfVertex": 8,
    "bindVertex": 0,
    "graph": {
        "0": [1, 2, 5],
        "1": [7, 3, 2],
        "2": [7, 3],
        "3": [4],
        "4": [6, 5],
        "5": [6],
        "6": [7],
    }
}

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


def filterList(listValues, valuesToFilter):
    toFilter = []
    if isinstance(valuesToFilter, list):
        toFilter = valuesToFilter
    else:
        toFilter.append(valuesToFilter)

    return list(x for x in listValues if x not in toFilter)


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


def removePair(adjacencies, v1, v2):
    newAdj = {}
    for k, v in adjacencies.items():
        if k == v1 or k == v2:
            continue
        newAdj[k] = filterList(v, [v1, v2])
    return newAdj


def removeEdge(adjacencies, v1, v2):
    newAdj = deepcopy(adjacencies)
    newAdj[v1] = filterList(newAdj[v1], v2)
    newAdj[v2] = filterList(newAdj[v2], v1)
    return newAdj


def isNeighbor(adjacencies, k1, k2):
    return k1 in adjacencies[k2]


def filterParing(adjacencies, PARINGS):
    paringsCopy = deepcopy(PARINGS)
    for p in paringsCopy:
        for v1, v2 in p:
            if not isNeighbor(adjacencies, v1, v2) and p in PARINGS:
                PARINGS.remove(p)


def paring(adjacencies, isParent, maxGroupOfPairs, PARINGS=None):
    if PARINGS is None:
        PARINGS = []
    adjacencyKeys = list(adjacencies.keys())
    # Nunca deveriamos entrar aqui
    if len(adjacencyKeys) < 2:
        print("GRAFO COM NUMERO IMPAR DE VERTICES")
        return

    if len(adjacencyKeys) == 2:
        PARINGS[-1].append((adjacencyKeys[0], adjacencyKeys[1]))
        return

    parentVertex = list(adjacencyKeys)[0]

    parentNeighbors = adjacencies[parentVertex]

    for n in parentNeighbors:
        newPair = (parentVertex, n)
        if isParent:
            PARINGS.append([newPair])
        else:
            lastParing = PARINGS[-1]
            if len(lastParing) == maxGroupOfPairs:
                PARINGS.append(lastParing[0:-2] + [newPair])
            else:
                lastParing.append(newPair)
        newAdj = removePair(adjacencies, parentVertex, n)
        paring(newAdj, False, maxGroupOfPairs, PARINGS)
    if isParent:
        filterParing(adjacencies, PARINGS)
        return PARINGS


def connectVertex(g, pair):
    v1, v2 = pair
    if v1 in g.keys():
        g[v1] += [v2]
    else:
        g[v1] = [v2]


def findCycle(adjacencies, vertex, paths=None, isParent=False):
    if paths is None:
        paths = [[]]

    if not len(adjacencies[vertex]):
        lastPath = paths[-1]
        if vertex == lastPath[0]:
            paths.append([vertex])
        else:
            paths.append(deepcopy(lastPath))
        lastPath.append(vertex)
        return
    paths[-1].append(vertex)
    for v in adjacencies[vertex]:
        newAdj = removeEdge(adjacencies, vertex, v)
        findCycle(newAdj, v, paths, False)
    if isParent:
        cycles = []
        for p in paths:
            if len(p) > 1 and p[0] == p[-1]:
                for c in cycles:
                    if sorted(c) == sorted(p):
                        break
                else:
                    cycles.append(p)
        return cycles


def countCycles(nVertices, paring1, paring2):
    g = {"numberOfVertex": nVertices, "bindVertex": paring1[0][0], "graph": {}}
    for p1, p2 in zip(paring1, paring2):
        connectVertex(g["graph"], p1)
        connectVertex(g["graph"], p2)
    graph = getGraph(g)

    degrees = graph.getVertexDegree()
    adjacencies = graph.getAllVertexAdjacency()

    count = 0
    lastCycle = []
    for v, d in degrees.items():
        if d > 1:
            foundCycles = findCycle(adjacencies, v, isParent=True)
            amountFoundCycles = len(foundCycles)
            if amountFoundCycles == 0:
                continue

            if amountFoundCycles > 1:
                break

            cycle = foundCycles[0]
            cycle.remove(v)
            if sorted(lastCycle) != sorted(cycle):
                lastCycle = cycle
                count += 1
    else:
        return count
    # se for encontrado mais de um ciclo para um vertice
    # já sabemos que não é PM compacto, então posso retorna qualquer número
    # maior que 1
    return 42


def isPMCompact(originGraph):
    adjacencies = originGraph.getAllVertexAdjacency()

    if originGraph.vertexAmount % 2 != 0:
        print("Numero de vertices não deveria ser impar")
        return False

    parings = paring(
        adjacencies=adjacencies,
        isParent=True,
        maxGroupOfPairs=len(adjacencies.keys()) / 2
    )
    for p1, index in zip(parings, range(1, len(parings))):
        for p2 in parings[index: -1]:
            result = countCycles(originGraph.vertexAmount, p1, p2)
            if result != 1:
                return False
    return True


def createWheel(size):
    g = {"numberOfVertex": size, "bindVertex": 0, "graph": {}}
    for i in range(1, size):
        nextV = i + 1
        if nextV == size:
            nextV = 1
        beforeV = i - 1
        if beforeV == 0:
            beforeV = size - 1
        connectVertex(g["graph"], (i, 0))
        connectVertex(g["graph"], (i, nextV))
        connectVertex(g["graph"], (i, beforeV))
    return getGraph(g)


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

    queues = {}
    for vertexNumber in range(4, 30, 2):
        currentQueue = queues[vertexNumber]
        if not currentQueue:
            currentQueue = []

        if vertexNumber == 4:
            currentQueue.append(getGraph(K4))
        if vertexNumber == 6:
            currentQueue.append(getGraph(K33))
        if vertexNumber == 8:
            currentQueue.append(getGraph(S8))

        if vertexNumber > 4:
            currentQueue.append(createWheel(vertexNumber))


    count = 1
    for g in ACCEPTED_GRAPHS:
        if isPMCompact(g):
            print("-=-==- GRAFO {} -=-==- ".format(count))
            print(g)
            drawGraph(g, "graph-{}".format(count))
            count += 1
    print("TOTAL GRAFOS GERADOS: {}".format(GRAPHS_COUNT))
    print("TOTAL GRAFOS ISOMORFOS: {}".format(ISOMORPHIC_GRAPH_COUNT))


main()
