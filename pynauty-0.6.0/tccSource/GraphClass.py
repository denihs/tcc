from pynauty import Graph


class GraphExt(Graph):
    def __init__(self, vertexAmount, bindVertex=None, isSolid=False, isK33=False):
        super().__init__(vertexAmount)
        self.vertexAmount = vertexAmount
        self.bindVertex = None
        self.bindVertex = bindVertex
        self.connections = {}
        self.id = None
        self.parents = None
        self.isSolid = isSolid
        self.isK33 = isK33

    def setBindVertex(self, v):
        self.bindVertex = v

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

    def getId(self):
        return self.id

    def setId(self, index):
        self.id = "{}-{}".format(self.vertexAmount, index)

    def getParents(self):
        return self.parents

    def setParents(self, p1, p2):
        self.parents = "{} | {}".format(p1, p2)

    def getInfo(self):
        return {
            "id": self.id,
            "parents": self.parents,
            "vertexAmount": self.vertexAmount,
            "bindVertex": self.bindVertex,
            "isSolid": self.isSolid,
            "isK33": self.isK33,
            "Adjacency": self.getAllVertexAdjacency(),
        }
