from pynauty import Graph


class GraphExt(Graph):
    def __init__(self, vertexAmount, bindVertex=None, bindDegree=None):
        super().__init__(vertexAmount)
        self.vertexAmount = vertexAmount
        self.bindVertex = bindVertex
        self.bindDegree = bindDegree
        self.connections = {}
        self.internalGraphEnd = None
        self.internalGraphVertexAmount = None
        self.externalGraphVertexAmount = None
        self.tier = None

    def getAdjacency(self):
        return self.adjacency_dict

    def _vertexDegree(self, v):
        adjacencyDict = self.getAdjacency()
        count = 0
        adjacencyList = []
        if v in adjacencyDict.keys():
            adjacencyList = adjacencyDict[v]
            count += len(adjacencyList)
        for key, listValues in adjacencyDict.items():
            if v in listValues and key not in adjacencyList:
                count += 1
        return count

    def getVertexDegree(self, v=None):
        allDegrees = {}
        if v is not None:

            if v > self.vertexAmount - 1:
                raise Exception("Número do vertice é maior do que o esperado.")

            return self._vertexDegree(v)

        for i in range(self.vertexAmount):
            allDegrees[i] = self._vertexDegree(i)
        return allDegrees

    def getVertexAdjacency(self, v):
        selfAdjacency = self.getAdjacency()
        adjacency = []

        if v in selfAdjacency.keys():
            adjacency = adjacency + selfAdjacency[v]

        for vertex in selfAdjacency.keys():
            values = selfAdjacency[vertex]
            if v in values and vertex not in adjacency:
                adjacency.append(vertex)
        return adjacency

    def getAllVertexAdjacency(self):
        adjacency = {}
        for i in range(self.vertexAmount):
            adjacency[i] = self.getVertexAdjacency(i)
        return adjacency

    def setConnections(self, v1, v2):
        self.connections[v1] = v2

    def setInternalGraphEnd(self, end):
        self.internalGraphEnd = end

    def setInternalGraphVertexAmount(self, amount):
        self.internalGraphVertexAmount = amount

    def setExternalGraphVertexAmount(self, amount):
        self.externalGraphVertexAmount = amount

    def setTier(self, tier):
        self.tier = tier
