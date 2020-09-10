from pynauty import Graph

class GraphExt(Graph):
    def __init__(self, vertexAmount):
        super().__init__(vertexAmount)
        self.vertexAmount = vertexAmount

    def getAdjacecy(self):
        return self.adjacency_dict

    def _vertexDegree(self, v):
        adjacencyDict = self.getAdjacecy()
        count = 0
        if v in adjacencyDict.keys():
            count += len(adjacencyDict[v])
        for a in adjacencyDict.values():
            if v in a:
                count += 1
        return count

    def getVertexDegree(self, v=None):
        allDegrees = {}
        if v is not None:

            if v > self.vertexAmount - 1:
                raise Exception("Número do vertice é maior do que o esperado.")

            allDegrees[v] = self._vertexDegree(v)
            return allDegrees

        for i in range(self.vertexAmount):
            allDegrees[i] = self._vertexDegree(i)
        return allDegrees