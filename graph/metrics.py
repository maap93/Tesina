# -*- coding: utf-8 -*-
from graph.nodes import node
from networkx import Graph
import math, numpy as np

__all__ = [ 'cost_function', 'heuristic' ]


def manhattan(start: node, next: node):
	return abs(start.x + next.x) + abs(start.y + next.y)

def euclidean(start: node, next: node) :
	return math.sqrt((next.x - start.x) ** 2 + (next.y - start.y) ** 2)

def cost_function(graph: Graph, current: node, next: node, costs: dict = None, **kwargs) :

	distance = manhattan(current, next)
	time = graph[current][next]['time']
	delay = next.delay


	if costs is not None: costs[next] = (distance, time, delay)

	return sum((distance, time, delay))

def heuristic(graph: Graph, current: node, goal: node, **kwargs) :

	distance = manhattan(current, goal)
	time = np.random.poisson(10.0)
	delay = np.random.binomial(10, 0.1)

	return distance + time + delay
