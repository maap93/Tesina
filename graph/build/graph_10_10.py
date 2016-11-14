# -*- coding: utf-8 -*-
from graph.nodes import node
from networkx import Graph
import numpy as np

__all__ = [ 'graph', 'nodes' ]


# Define the graph and nodes
graph = Graph(name = 'Grid 10x10')

# Generate a set of nodes from 0 .. width - 1 and 0 .. height - 1
nodes = [
    node(0, 0),
    node(0, 1),
    node(0, 2),
    node(0, 3),
    node(0, 4),
    node(0, 5),
    node(0, 6),
    node(0, 7),
    node(0, 8),
    node(0, 9),
    node(1, 0),
    node(1, 1),
    node(1, 2),
    node(1, 3),
    node(1, 4),
    node(1, 5),
    node(1, 6),
    node(1, 7),
    node(1, 8),
    node(1, 9),
    node(2, 0),
    node(2, 1),
    node(2, 2),
    node(2, 3),
    node(2, 4),
    node(2, 5),
    node(2, 6),
    node(2, 7),
    node(2, 8),
    node(2, 9),
    node(3, 0),
    node(3, 1),
    node(3, 2),
    node(3, 3),
    node(3, 4),
    node(3, 5),
    node(3, 6),
    node(3, 7),
    node(3, 8),
    node(3, 9),
    node(4, 0),
    node(4, 1),
    node(4, 2),
    node(4, 3),
    node(4, 4),
    node(4, 5),
    node(4, 6),
    node(4, 7),
    node(4, 8),
    node(4, 9),
    node(5, 0),
    node(5, 1),
    node(5, 2),
    node(5, 3),
    node(5, 4),
    node(5, 5),
    node(5, 6),
    node(5, 7),
    node(5, 8),
    node(5, 9),
    node(6, 0),
    node(6, 1),
    node(6, 2),
    node(6, 3),
    node(6, 4),
    node(6, 5),
    node(6, 6),
    node(6, 7),
    node(6, 8),
    node(6, 9),
    node(7, 0),
    node(7, 1),
    node(7, 2),
    node(7, 3),
    node(7, 4),
    node(7, 5),
    node(7, 6),
    node(7, 7),
    node(7, 8),
    node(7, 9),
    node(8, 0),
    node(8, 1),
    node(8, 2),
    node(8, 3),
    node(8, 4),
    node(8, 5),
    node(8, 6),
    node(8, 7),
    node(8, 8),
    node(8, 9),
    node(9, 0),
    node(9, 1),
    node(9, 2),
    node(9, 3),
    node(9, 4),
    node(9, 5),
    node(9, 6),
    node(9, 7),
    node(9, 8),
    node(9, 9),
]
graph.add_nodes_from(nodes)

# Generate a set of edges connecting each node in the list on an inverted L pattern
graph.add_edges_from([
    # For node (0, 0)
    (nodes[0], nodes[10], { 'time': np.random.poisson(10.0) }),
    (nodes[0], nodes[1], { 'time': np.random.poisson(10.0) }),
    # For node (0, 1)
    (nodes[1], nodes[11], { 'time': np.random.poisson(10.0) }),
    (nodes[1], nodes[2], { 'time': np.random.poisson(10.0) }),
    # For node (0, 2)
    (nodes[2], nodes[12], { 'time': np.random.poisson(10.0) }),
    (nodes[2], nodes[3], { 'time': np.random.poisson(10.0) }),
    # For node (0, 3)
    (nodes[3], nodes[13], { 'time': np.random.poisson(10.0) }),
    (nodes[3], nodes[4], { 'time': np.random.poisson(10.0) }),
    # For node (0, 4)
    (nodes[4], nodes[14], { 'time': np.random.poisson(10.0) }),
    (nodes[4], nodes[5], { 'time': np.random.poisson(10.0) }),
    # For node (0, 5)
    (nodes[5], nodes[15], { 'time': np.random.poisson(10.0) }),
    (nodes[5], nodes[6], { 'time': np.random.poisson(10.0) }),
    # For node (0, 6)
    (nodes[6], nodes[16], { 'time': np.random.poisson(10.0) }),
    (nodes[6], nodes[7], { 'time': np.random.poisson(10.0) }),
    # For node (0, 7)
    (nodes[7], nodes[17], { 'time': np.random.poisson(10.0) }),
    (nodes[7], nodes[8], { 'time': np.random.poisson(10.0) }),
    # For node (0, 8)
    (nodes[8], nodes[18], { 'time': np.random.poisson(10.0) }),
    (nodes[8], nodes[9], { 'time': np.random.poisson(10.0) }),
    # For node (0, 9)
    (nodes[9], nodes[19], { 'time': np.random.poisson(10.0) }),
    # For node (1, 0)
    (nodes[10], nodes[20], { 'time': np.random.poisson(10.0) }),
    (nodes[10], nodes[11], { 'time': np.random.poisson(10.0) }),
    # For node (1, 1)
    (nodes[11], nodes[21], { 'time': np.random.poisson(10.0) }),
    (nodes[11], nodes[12], { 'time': np.random.poisson(10.0) }),
    # For node (1, 2)
    (nodes[12], nodes[22], { 'time': np.random.poisson(10.0) }),
    (nodes[12], nodes[13], { 'time': np.random.poisson(10.0) }),
    # For node (1, 3)
    (nodes[13], nodes[23], { 'time': np.random.poisson(10.0) }),
    (nodes[13], nodes[14], { 'time': np.random.poisson(10.0) }),
    # For node (1, 4)
    (nodes[14], nodes[24], { 'time': np.random.poisson(10.0) }),
    (nodes[14], nodes[15], { 'time': np.random.poisson(10.0) }),
    # For node (1, 5)
    (nodes[15], nodes[25], { 'time': np.random.poisson(10.0) }),
    (nodes[15], nodes[16], { 'time': np.random.poisson(10.0) }),
    # For node (1, 6)
    (nodes[16], nodes[26], { 'time': np.random.poisson(10.0) }),
    (nodes[16], nodes[17], { 'time': np.random.poisson(10.0) }),
    # For node (1, 7)
    (nodes[17], nodes[27], { 'time': np.random.poisson(10.0) }),
    (nodes[17], nodes[18], { 'time': np.random.poisson(10.0) }),
    # For node (1, 8)
    (nodes[18], nodes[28], { 'time': np.random.poisson(10.0) }),
    (nodes[18], nodes[19], { 'time': np.random.poisson(10.0) }),
    # For node (1, 9)
    (nodes[19], nodes[29], { 'time': np.random.poisson(10.0) }),
    # For node (2, 0)
    (nodes[20], nodes[30], { 'time': np.random.poisson(10.0) }),
    (nodes[20], nodes[21], { 'time': np.random.poisson(10.0) }),
    # For node (2, 1)
    (nodes[21], nodes[31], { 'time': np.random.poisson(10.0) }),
    (nodes[21], nodes[22], { 'time': np.random.poisson(10.0) }),
    # For node (2, 2)
    (nodes[22], nodes[32], { 'time': np.random.poisson(10.0) }),
    (nodes[22], nodes[23], { 'time': np.random.poisson(10.0) }),
    # For node (2, 3)
    (nodes[23], nodes[33], { 'time': np.random.poisson(10.0) }),
    (nodes[23], nodes[24], { 'time': np.random.poisson(10.0) }),
    # For node (2, 4)
    (nodes[24], nodes[34], { 'time': np.random.poisson(10.0) }),
    (nodes[24], nodes[25], { 'time': np.random.poisson(10.0) }),
    # For node (2, 5)
    (nodes[25], nodes[35], { 'time': np.random.poisson(10.0) }),
    (nodes[25], nodes[26], { 'time': np.random.poisson(10.0) }),
    # For node (2, 6)
    (nodes[26], nodes[36], { 'time': np.random.poisson(10.0) }),
    (nodes[26], nodes[27], { 'time': np.random.poisson(10.0) }),
    # For node (2, 7)
    (nodes[27], nodes[37], { 'time': np.random.poisson(10.0) }),
    (nodes[27], nodes[28], { 'time': np.random.poisson(10.0) }),
    # For node (2, 8)
    (nodes[28], nodes[38], { 'time': np.random.poisson(10.0) }),
    (nodes[28], nodes[29], { 'time': np.random.poisson(10.0) }),
    # For node (2, 9)
    (nodes[29], nodes[39], { 'time': np.random.poisson(10.0) }),
    # For node (3, 0)
    (nodes[30], nodes[40], { 'time': np.random.poisson(10.0) }),
    (nodes[30], nodes[31], { 'time': np.random.poisson(10.0) }),
    # For node (3, 1)
    (nodes[31], nodes[41], { 'time': np.random.poisson(10.0) }),
    (nodes[31], nodes[32], { 'time': np.random.poisson(10.0) }),
    # For node (3, 2)
    (nodes[32], nodes[42], { 'time': np.random.poisson(10.0) }),
    (nodes[32], nodes[33], { 'time': np.random.poisson(10.0) }),
    # For node (3, 3)
    (nodes[33], nodes[43], { 'time': np.random.poisson(10.0) }),
    (nodes[33], nodes[34], { 'time': np.random.poisson(10.0) }),
    # For node (3, 4)
    (nodes[34], nodes[44], { 'time': np.random.poisson(10.0) }),
    (nodes[34], nodes[35], { 'time': np.random.poisson(10.0) }),
    # For node (3, 5)
    (nodes[35], nodes[45], { 'time': np.random.poisson(10.0) }),
    (nodes[35], nodes[36], { 'time': np.random.poisson(10.0) }),
    # For node (3, 6)
    (nodes[36], nodes[46], { 'time': np.random.poisson(10.0) }),
    (nodes[36], nodes[37], { 'time': np.random.poisson(10.0) }),
    # For node (3, 7)
    (nodes[37], nodes[47], { 'time': np.random.poisson(10.0) }),
    (nodes[37], nodes[38], { 'time': np.random.poisson(10.0) }),
    # For node (3, 8)
    (nodes[38], nodes[48], { 'time': np.random.poisson(10.0) }),
    (nodes[38], nodes[39], { 'time': np.random.poisson(10.0) }),
    # For node (3, 9)
    (nodes[39], nodes[49], { 'time': np.random.poisson(10.0) }),
    # For node (4, 0)
    (nodes[40], nodes[50], { 'time': np.random.poisson(10.0) }),
    (nodes[40], nodes[41], { 'time': np.random.poisson(10.0) }),
    # For node (4, 1)
    (nodes[41], nodes[51], { 'time': np.random.poisson(10.0) }),
    (nodes[41], nodes[42], { 'time': np.random.poisson(10.0) }),
    # For node (4, 2)
    (nodes[42], nodes[52], { 'time': np.random.poisson(10.0) }),
    (nodes[42], nodes[43], { 'time': np.random.poisson(10.0) }),
    # For node (4, 3)
    (nodes[43], nodes[53], { 'time': np.random.poisson(10.0) }),
    (nodes[43], nodes[44], { 'time': np.random.poisson(10.0) }),
    # For node (4, 4)
    (nodes[44], nodes[54], { 'time': np.random.poisson(10.0) }),
    (nodes[44], nodes[45], { 'time': np.random.poisson(10.0) }),
    # For node (4, 5)
    (nodes[45], nodes[55], { 'time': np.random.poisson(10.0) }),
    (nodes[45], nodes[46], { 'time': np.random.poisson(10.0) }),
    # For node (4, 6)
    (nodes[46], nodes[56], { 'time': np.random.poisson(10.0) }),
    (nodes[46], nodes[47], { 'time': np.random.poisson(10.0) }),
    # For node (4, 7)
    (nodes[47], nodes[57], { 'time': np.random.poisson(10.0) }),
    (nodes[47], nodes[48], { 'time': np.random.poisson(10.0) }),
    # For node (4, 8)
    (nodes[48], nodes[58], { 'time': np.random.poisson(10.0) }),
    (nodes[48], nodes[49], { 'time': np.random.poisson(10.0) }),
    # For node (4, 9)
    (nodes[49], nodes[59], { 'time': np.random.poisson(10.0) }),
    # For node (5, 0)
    (nodes[50], nodes[60], { 'time': np.random.poisson(10.0) }),
    (nodes[50], nodes[51], { 'time': np.random.poisson(10.0) }),
    # For node (5, 1)
    (nodes[51], nodes[61], { 'time': np.random.poisson(10.0) }),
    (nodes[51], nodes[52], { 'time': np.random.poisson(10.0) }),
    # For node (5, 2)
    (nodes[52], nodes[62], { 'time': np.random.poisson(10.0) }),
    (nodes[52], nodes[53], { 'time': np.random.poisson(10.0) }),
    # For node (5, 3)
    (nodes[53], nodes[63], { 'time': np.random.poisson(10.0) }),
    (nodes[53], nodes[54], { 'time': np.random.poisson(10.0) }),
    # For node (5, 4)
    (nodes[54], nodes[64], { 'time': np.random.poisson(10.0) }),
    (nodes[54], nodes[55], { 'time': np.random.poisson(10.0) }),
    # For node (5, 5)
    (nodes[55], nodes[65], { 'time': np.random.poisson(10.0) }),
    (nodes[55], nodes[56], { 'time': np.random.poisson(10.0) }),
    # For node (5, 6)
    (nodes[56], nodes[66], { 'time': np.random.poisson(10.0) }),
    (nodes[56], nodes[57], { 'time': np.random.poisson(10.0) }),
    # For node (5, 7)
    (nodes[57], nodes[67], { 'time': np.random.poisson(10.0) }),
    (nodes[57], nodes[58], { 'time': np.random.poisson(10.0) }),
    # For node (5, 8)
    (nodes[58], nodes[68], { 'time': np.random.poisson(10.0) }),
    (nodes[58], nodes[59], { 'time': np.random.poisson(10.0) }),
    # For node (5, 9)
    (nodes[59], nodes[69], { 'time': np.random.poisson(10.0) }),
    # For node (6, 0)
    (nodes[60], nodes[70], { 'time': np.random.poisson(10.0) }),
    (nodes[60], nodes[61], { 'time': np.random.poisson(10.0) }),
    # For node (6, 1)
    (nodes[61], nodes[71], { 'time': np.random.poisson(10.0) }),
    (nodes[61], nodes[62], { 'time': np.random.poisson(10.0) }),
    # For node (6, 2)
    (nodes[62], nodes[72], { 'time': np.random.poisson(10.0) }),
    (nodes[62], nodes[63], { 'time': np.random.poisson(10.0) }),
    # For node (6, 3)
    (nodes[63], nodes[73], { 'time': np.random.poisson(10.0) }),
    (nodes[63], nodes[64], { 'time': np.random.poisson(10.0) }),
    # For node (6, 4)
    (nodes[64], nodes[74], { 'time': np.random.poisson(10.0) }),
    (nodes[64], nodes[65], { 'time': np.random.poisson(10.0) }),
    # For node (6, 5)
    (nodes[65], nodes[75], { 'time': np.random.poisson(10.0) }),
    (nodes[65], nodes[66], { 'time': np.random.poisson(10.0) }),
    # For node (6, 6)
    (nodes[66], nodes[76], { 'time': np.random.poisson(10.0) }),
    (nodes[66], nodes[67], { 'time': np.random.poisson(10.0) }),
    # For node (6, 7)
    (nodes[67], nodes[77], { 'time': np.random.poisson(10.0) }),
    (nodes[67], nodes[68], { 'time': np.random.poisson(10.0) }),
    # For node (6, 8)
    (nodes[68], nodes[78], { 'time': np.random.poisson(10.0) }),
    (nodes[68], nodes[69], { 'time': np.random.poisson(10.0) }),
    # For node (6, 9)
    (nodes[69], nodes[79], { 'time': np.random.poisson(10.0) }),
    # For node (7, 0)
    (nodes[70], nodes[80], { 'time': np.random.poisson(10.0) }),
    (nodes[70], nodes[71], { 'time': np.random.poisson(10.0) }),
    # For node (7, 1)
    (nodes[71], nodes[81], { 'time': np.random.poisson(10.0) }),
    (nodes[71], nodes[72], { 'time': np.random.poisson(10.0) }),
    # For node (7, 2)
    (nodes[72], nodes[82], { 'time': np.random.poisson(10.0) }),
    (nodes[72], nodes[73], { 'time': np.random.poisson(10.0) }),
    # For node (7, 3)
    (nodes[73], nodes[83], { 'time': np.random.poisson(10.0) }),
    (nodes[73], nodes[74], { 'time': np.random.poisson(10.0) }),
    # For node (7, 4)
    (nodes[74], nodes[84], { 'time': np.random.poisson(10.0) }),
    (nodes[74], nodes[75], { 'time': np.random.poisson(10.0) }),
    # For node (7, 5)
    (nodes[75], nodes[85], { 'time': np.random.poisson(10.0) }),
    (nodes[75], nodes[76], { 'time': np.random.poisson(10.0) }),
    # For node (7, 6)
    (nodes[76], nodes[86], { 'time': np.random.poisson(10.0) }),
    (nodes[76], nodes[77], { 'time': np.random.poisson(10.0) }),
    # For node (7, 7)
    (nodes[77], nodes[87], { 'time': np.random.poisson(10.0) }),
    (nodes[77], nodes[78], { 'time': np.random.poisson(10.0) }),
    # For node (7, 8)
    (nodes[78], nodes[88], { 'time': np.random.poisson(10.0) }),
    (nodes[78], nodes[79], { 'time': np.random.poisson(10.0) }),
    # For node (7, 9)
    (nodes[79], nodes[89], { 'time': np.random.poisson(10.0) }),
    # For node (8, 0)
    (nodes[80], nodes[90], { 'time': np.random.poisson(10.0) }),
    (nodes[80], nodes[81], { 'time': np.random.poisson(10.0) }),
    # For node (8, 1)
    (nodes[81], nodes[91], { 'time': np.random.poisson(10.0) }),
    (nodes[81], nodes[82], { 'time': np.random.poisson(10.0) }),
    # For node (8, 2)
    (nodes[82], nodes[92], { 'time': np.random.poisson(10.0) }),
    (nodes[82], nodes[83], { 'time': np.random.poisson(10.0) }),
    # For node (8, 3)
    (nodes[83], nodes[93], { 'time': np.random.poisson(10.0) }),
    (nodes[83], nodes[84], { 'time': np.random.poisson(10.0) }),
    # For node (8, 4)
    (nodes[84], nodes[94], { 'time': np.random.poisson(10.0) }),
    (nodes[84], nodes[85], { 'time': np.random.poisson(10.0) }),
    # For node (8, 5)
    (nodes[85], nodes[95], { 'time': np.random.poisson(10.0) }),
    (nodes[85], nodes[86], { 'time': np.random.poisson(10.0) }),
    # For node (8, 6)
    (nodes[86], nodes[96], { 'time': np.random.poisson(10.0) }),
    (nodes[86], nodes[87], { 'time': np.random.poisson(10.0) }),
    # For node (8, 7)
    (nodes[87], nodes[97], { 'time': np.random.poisson(10.0) }),
    (nodes[87], nodes[88], { 'time': np.random.poisson(10.0) }),
    # For node (8, 8)
    (nodes[88], nodes[98], { 'time': np.random.poisson(10.0) }),
    (nodes[88], nodes[89], { 'time': np.random.poisson(10.0) }),
    # For node (8, 9)
    (nodes[89], nodes[99], { 'time': np.random.poisson(10.0) }),
    # For node (9, 0)
    (nodes[90], nodes[91], { 'time': np.random.poisson(10.0) }),
    # For node (9, 1)
    (nodes[91], nodes[92], { 'time': np.random.poisson(10.0) }),
    # For node (9, 2)
    (nodes[92], nodes[93], { 'time': np.random.poisson(10.0) }),
    # For node (9, 3)
    (nodes[93], nodes[94], { 'time': np.random.poisson(10.0) }),
    # For node (9, 4)
    (nodes[94], nodes[95], { 'time': np.random.poisson(10.0) }),
    # For node (9, 5)
    (nodes[95], nodes[96], { 'time': np.random.poisson(10.0) }),
    # For node (9, 6)
    (nodes[96], nodes[97], { 'time': np.random.poisson(10.0) }),
    # For node (9, 7)
    (nodes[97], nodes[98], { 'time': np.random.poisson(10.0) }),
    # For node (9, 8)
    (nodes[98], nodes[99], { 'time': np.random.poisson(10.0) }),
    # For node (9, 9)
])