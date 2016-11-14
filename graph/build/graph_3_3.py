# -*- coding: utf-8 -*-
from graph.nodes import node
from networkx import Graph
import numpy as np

__all__ = [ 'graph', 'nodes' ]


# Define the graph and nodes
graph = Graph(name = 'Grid 3x3')

# Generate a set of nodes from 0 .. width - 1 and 0 .. height - 1
nodes = [
    node(0, 0),
    node(0, 1),
    node(0, 2),
    node(1, 0),
    node(1, 1),
    node(1, 2),
    node(2, 0),
    node(2, 1),
    node(2, 2),
]
graph.add_nodes_from(nodes)

# Generate a set of edges connecting each node in the list on an inverted L pattern
graph.add_edges_from([
    # For node (0, 0)
    (nodes[0], nodes[3], { 'time': np.random.poisson(10.0) }),
    (nodes[0], nodes[1], { 'time': np.random.poisson(10.0) }),
    # For node (0, 1)
    (nodes[1], nodes[4], { 'time': np.random.poisson(10.0) }),
    (nodes[1], nodes[2], { 'time': np.random.poisson(10.0) }),
    # For node (0, 2)
    (nodes[2], nodes[5], { 'time': np.random.poisson(10.0) }),
    # For node (1, 0)
    (nodes[3], nodes[6], { 'time': np.random.poisson(10.0) }),
    (nodes[3], nodes[4], { 'time': np.random.poisson(10.0) }),
    # For node (1, 1)
    (nodes[4], nodes[7], { 'time': np.random.poisson(10.0) }),
    (nodes[4], nodes[5], { 'time': np.random.poisson(10.0) }),
    # For node (1, 2)
    (nodes[5], nodes[8], { 'time': np.random.poisson(10.0) }),
    # For node (2, 0)
    (nodes[6], nodes[7], { 'time': np.random.poisson(10.0) }),
    # For node (2, 1)
    (nodes[7], nodes[8], { 'time': np.random.poisson(10.0) }),
    # For node (2, 2)
])