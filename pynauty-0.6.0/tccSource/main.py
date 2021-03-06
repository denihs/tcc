from pynauty import isomorphic, autgrp
import json
import os
from GraphClass import GraphExt
from copy import deepcopy
from itertools import permutations
import igraph

GRAPHS_COUNT = 0
ISOMORPHIC_GRAPH_COUNT = 0
QUEUES_AMOUNT = 30

K4 = {
    "numberOfVertex": 4,
    "isSolid": True,
    "graph": {
        "0": [1, 2, 3],
        "1": [2, 3],
        "2": [3],
    }
}

K33 = {
    "numberOfVertex": 6,
    "bindVertex": 0,
    "isSolid": True,
    "isK33": True,
    "graph": {
        "0": [3, 4, 5],
        "1": [3, 4, 5],
        "2": [3, 4, 5],
    }
}

S8 = {
    "numberOfVertex": 8,
    "isSolid": True,
    "graph": {
        "0": [1, 2, 3, 4],
        "1": [0, 2, 3, 4],
        "2": [0, 1, 7],
        "3": [0, 1, 6],
        "4": [0, 1, 5],
        "5": [4, 6, 7],
        "6": [3, 5, 7],
        "7": [2, 5, 6],
    }
}

WANG = {
    "numberOfVertex": 12,
    "isSolid": True,
    "graph": {
        "0": [1, 4, 7],
        "1": [0, 2, 9],
        "2": [1, 3, 10],
        "3": [2, 4, 11],
        "4": [0, 3, 5],
        "5": [4, 6, 10],
        "6": [5, 7, 11],
        "7": [0, 6, 8],
        "8": [7, 9, 10],
        "9": [1, 8, 11],
        "10": [2, 5, 8],
        "11": [3, 6, 9],
    }
}


def getIsIsomorphic(g, h):
    return isomorphic(g, h)


def getGraph(data):
    isSolid = None
    isK33 = None
    bindVertex = None
    if "isSolid" in data.keys():
        isSolid = data["isSolid"]
    if "isK33" in data.keys():
        isK33 = data["isK33"]
    if "bindVertex" in data.keys():
        bindVertex = data["bindVertex"]

    g = GraphExt(vertexAmount=data["numberOfVertex"], isSolid=isSolid, isK33=isK33, bindVertex=bindVertex)
    graph = data["graph"]
    for v in graph:
        g.connect_vertex(int(v), graph[v])
    return g


# Esse cara eh especial, vamos trata-lo diferente no codigo
K33_GRAPH = getGraph(K33)


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


def findBindVertex(g1, g2):
    d1 = g1.getVertexDegree()
    d2 = g2.getVertexDegree()

    for k1, v1 in d1.items():
        for k2, v2 in d2.items():
            if v1 == v2:
                return k1, k2
    return None, None


def getGraphNewBindVertex(graph, graphToBind, mapValues):
    if graph.isK33:
        if graph.bindVertex < 3:
            newBindVertex = filterList([3, 4, 5], graph.bindVertex)[0]
        else:
            newBindVertex = filterList([0, 1, 2], graph.bindVertex)[0]
        graphToBind.setBindVertex(list(mapValues.keys())[list(mapValues.values()).index(newBindVertex)])


def bindGraph(g1, g2):
    if g1.vertexAmount < g2.vertexAmount:
        graph1 = g1
        graph2 = g2
    else:
        graph1 = g2
        graph2 = g1

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
    getGraphNewBindVertex(graph1, g, map1)
    getGraphNewBindVertex(graph2, g, map2)

    # conectando as partes dos grafos
    # descobre quem está com grau faltando no primeiro grafo
    # e adiciona uma aresta para todos os vértices dele
    gDegrees = g.getVertexDegree()
    graph1Degrees = graph1.getVertexDegree()
    graph2Degrees = graph2.getVertexDegree()
    diffs = {}
    diffs2 = []
    for key, degree in gDegrees.items():
        if key < graph1.vertexAmount - 1:
            if degree < graph1Degrees[map1[key]]:
                diffs[key] = graph1Degrees[map1[key]] - degree
        else:
            if degree < graph2Degrees[map2[key]]:
                diffs2.append(key)

    for key, diff in diffs.items():
        if key < graph1.vertexAmount - 1:
            p = diffs2.pop()
            for i in range(diff):
                g.setConnections(key, p)
                g.connect_vertex(key, [p] + g.adjacency_dict[key])

    return g


def getVerticesByOrbit(orbits):
    verticesByOrbit = {}
    vertexCount = 0
    for orbit in orbits:
        verticesByOrbit[orbit] = vertexCount
        vertexCount += 1
    return verticesByOrbit


def bindByOrbits(graph1, graph2):
    bondedGraphs = []
    orbits1 = autgrp(graph1)[3]
    orbits2 = autgrp(graph2)[3]

    verticesByOrbit1 = getVerticesByOrbit(orbits1)
    verticesByOrbit2 = getVerticesByOrbit(orbits2)
    for vertex1 in verticesByOrbit1.values():
        for vertex2 in verticesByOrbit2.values():
            g1VertexDegree = graph1.getVertexDegree(vertex1)
            g2VertexDegree = graph2.getVertexDegree(vertex2)
            if g1VertexDegree != g2VertexDegree:
                continue

            g1 = deepcopy(graph1)
            g2 = deepcopy(graph2)

            g1.setBindVertex(vertex1)
            g2.setBindVertex(vertex2)

            bondedGraphs.append(bindGraph(g1, g2))
    return bondedGraphs


def twistEdges(graph, alreadyAccepted):
    newAcceptedGraphs = []
    connections = graph.connections

    keys = list(connections.keys())
    perm = list(permutations(connections.values()))

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
        if not isPMCompact(g):
            continue
        for acceptedGraph in alreadyAccepted + newAcceptedGraphs:
            if getIsIsomorphic(g, acceptedGraph):
                break
        else:
            newAcceptedGraphs.append(g)
    return newAcceptedGraphs


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


def vertexShowsTwice(parings, v1, v2):
    count = 0
    for p in parings:
        if v1 in p or v2 in p:
            count += 1
    return count > 1


def filterParing(adjacencies, PARINGS):
    paringsCopy = deepcopy(PARINGS)
    parings = deepcopy(PARINGS)
    for paringsGroup in paringsCopy:
        for p in paringsGroup:
            v1, v2 = p
            if not isNeighbor(adjacencies, v1, v2) or vertexShowsTwice(paringsGroup, v1, v2):
                parings.remove(paringsGroup)
    if len(parings) == 0:
        return [[]]
    return parings


def paring(adjacencies, PARINGS=None, listParing=None):
    if PARINGS is None:
        PARINGS = []
    if listParing is None:
        listParing = []
    adjacencyKeys = list(adjacencies.keys())
    # Nunca deveriamos entrar aqui
    if len(adjacencyKeys) == 0:
        return
    if len(adjacencyKeys) == 1:
        print("GRAFO COM NUMERO IMPAR DE VERTICES")
        return

    if len(adjacencyKeys) == 2:
        PARINGS.append(listParing + [(adjacencyKeys[0], adjacencyKeys[1])])
        return

    parentVertex = list(adjacencyKeys)[0]

    parentNeighbors = adjacencies[parentVertex]
    for n in parentNeighbors:
        newAdj = removePair(adjacencies, parentVertex, n)
        newPair = (parentVertex, n)
        newList = deepcopy(listParing) + [newPair]
        paring(newAdj, PARINGS, newList)

    if parentVertex == 0:
        PARINGS = filterParing(adjacencies, PARINGS, )
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
    )
    result = 0

    paringsLen = len(parings)

    if paringsLen < 2:
        return False

    for p1, index in zip(parings, range(paringsLen)):
        for p2 in parings[index + 1:]:
            result = countCycles(originGraph.vertexAmount, p1, p2)
            if result != 1:
                return False
    # Nunca deveria acontecer isso aqui, mas just in case...
    if result == 0:
        print("WARNING - Resultado igual a ZERO na função isPMCompact")
        return False
    return True


def createWheel(size):
    g = {"numberOfVertex": size, "bindVertex": 0, "isSolid": True, "graph": {}}
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


def append_record(record, name):
    with open('./lists/{}'.format(name), 'a') as f:
        json.dump(record, f, indent=4)
        f.write(os.linesep)


def isThreeRegular(graph):
    graphDegree = graph.getVertexDegree()
    degrees = list(graphDegree.values())
    return degrees.count(degrees[0]) == len(degrees) and degrees[0] == 3


def registerGraph(graph, isGraphPmCompactAndThreeRegular=False):
    gId = graph.getId()
    if isGraphPmCompactAndThreeRegular:
        drawGraph(graph, "{}".format(gId))
        append_record(graph.getInfo(), "{}-compact".format(gId.split("-")[0]))
    append_record(graph.getInfo(), "{}".format(gId.split("-")[0]))


def initiateQueues():
    queues = {}
    for vertexNumber in range(4, QUEUES_AMOUNT, 2):
        queues[vertexNumber] = []
        currentQueue = queues[vertexNumber]

        if vertexNumber == 4:
            currentQueue.append(getGraph(K4))
        if vertexNumber == 6:
            currentQueue.append(K33_GRAPH)
        if vertexNumber == 8:
            currentQueue.append(getGraph(S8))
        if vertexNumber == 12:
            currentQueue.append(getGraph(WANG))

        if vertexNumber > 4:
            currentQueue.append(createWheel(vertexNumber))
    for graphs in queues.values():
        count = 0
        for g in graphs:
            g.setId(count)
            registerGraph(g)
            count += 1
    return queues


def selfPermute(queue, queues, alreadyAcceptedGraphs, alreadyBondedGraph=None):
    graphs = deepcopy(queue)
    g, h = graphs
    bondedGraphs = []
    if alreadyBondedGraph is None:
        bondedGraphs = bindByOrbits(g, h)
    else:
        bondedGraphs.append(alreadyBondedGraph)
    for bondedGraph in bondedGraphs:
        acceptedGraphs = twistEdges(bondedGraph, alreadyAcceptedGraphs)
        vertexAmount = bondedGraph.vertexAmount
        for acc in acceptedGraphs:
            alreadyAcceptedGraphs.append(acc)
            queues[vertexAmount].append(acc)
            acc.setId(len(queues[vertexAmount]) - 1)
            acc.setParents(g.getId(), h.getId())
            registerGraph(acc, isThreeRegular(acc))


def getVertexDegree3ToBind(g):
    verticesDegrees = g.getVertexDegree()
    vertexToBind = None
    for x in list(verticesDegrees.keys()):
        if verticesDegrees[x] == 3:
            vertexToBind = x
            break
    return vertexToBind


def calculateK33(g, queues, alreadyAcceptedGraphs, calculating):
    if calculating < 10:
        return

    # Essa conta sai da ideia que colar um grafo em cada partiçao do K33
    # a conta saida da expressao N = 6 + g.vertexAmount + h.vertexAmount - 4
    X = g.vertexAmount
    Y = calculating - 6 + 4 - X

    if Y < 4:
        return
    vertexToBind = getVertexDegree3ToBind(g)
    if vertexToBind is None:
        return

    g.setBindVertex(vertexToBind)
    firstGraph = bindGraph(deepcopy(K33_GRAPH), deepcopy(g))

    for graph in queues[Y]:
        if graph.isSolid:
            deepGraph = deepcopy(graph)

            vertexToBind = getVertexDegree3ToBind(deepGraph)
            deepGraph.setBindVertex(vertexToBind)

            bondedGraph = bindGraph(deepcopy(firstGraph), deepcopy(deepGraph))
            bondedGraph.setId("{} - {} * {}".format(calculating, firstGraph.getId(), deepGraph.getId()))
            bondedGraph.setParents(firstGraph.getId(), deepGraph.getId())
            selfPermute([deepcopy(bondedGraph), deepcopy(deepGraph)], queues, alreadyAcceptedGraphs, deepcopy(bondedGraph))
        

def computeLists(solids, notSolids, queues, alreadyAcceptedGraphs, calculating):
    firstSolid = solids[0]

    calculateK33(deepcopy(firstSolid), queues, alreadyAcceptedGraphs, calculating)

    for g in solids:
        for h in notSolids:
            if g.isK33 or h.isK33:
                continue
            selfPermute([deepcopy(g), deepcopy(h)], queues, alreadyAcceptedGraphs)


def permuteQueues(first, lastQueue, queues, alreadyAcceptedGraphs, calculating):
    # k33 eh um caso a parte
    firstSolids = [x for x in first if x.isSolid and not x.isK33]
    computeLists(firstSolids, lastQueue, queues, alreadyAcceptedGraphs, calculating)

    if first[0].vertexAmount == lastQueue[0].vertexAmount:
        return
    # k33 eh um caso a parte
    lastSolids = [x for x in lastQueue if x.isSolid and not x.isK33]
    computeLists(lastSolids, first, queues, alreadyAcceptedGraphs, calculating)


def main():
    global GRAPHS_COUNT
    global ISOMORPHIC_GRAPH_COUNT

    queues = initiateQueues()

    for vertexNumber in range(4, QUEUES_AMOUNT, 2):
        calculating = vertexNumber + 2
        rangeValues = list(range(4, calculating, 2))
        print("\n\n -=-=-=-=-=- CALCULANDO {} -=-=-=-=-=-".format(calculating))
        print("rangeValues - {}".format(rangeValues))
        alreadyAcceptedGraphs = []
        while len(rangeValues):
            if len(rangeValues) == 1:
                solo = rangeValues[0]
                print("solo - {} -> {}".format(solo, len(queues[solo])))
                permuteQueues(queues[solo], queues[solo], queues, alreadyAcceptedGraphs, calculating)
                rangeValues.pop(0)
                continue
            first = rangeValues.pop(0)
            last = rangeValues.pop(-1)
            print("first - {} -> {} | last - {} -> {}".format(first, len(queues[first]), last, len(queues[last])))
            permuteQueues(queues[first], queues[last], queues, alreadyAcceptedGraphs, calculating)

main()
