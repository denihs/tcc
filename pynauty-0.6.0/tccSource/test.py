from pynauty import *

g = Graph(5)
g.connect_vertex(0, [1, 2, 3])
g.connect_vertex(2, [1, 3, 4])
g.connect_vertex(4, [3])

h = Graph(5)
h.connect_vertex(0, [1, 3, 4])
h.connect_vertex(2, [3, 4])
h.connect_vertex(4, [3])
h.connect_vertex(3, [1])

print(certificate(g))
print(certificate(h))

print(isomorphic(g, h))