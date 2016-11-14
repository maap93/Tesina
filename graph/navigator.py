# -*- coding: utf-8 -*-
from graph.metrics import euclidean, manhattan
from queue import PriorityQueue
from graph.nodes import node
from networkx import Graph
from typing import *

__all__ = [ 'navigator', 'astar', 'dijkstra', 'bellman' ]


class empty: pass

# Class that contains the algorithms
class navigator(object):

	def __call__(self, graph: Graph, start: node, goal: node, **kwargs):
		raise NotImplementedError()

	@staticmethod
	def nearby(nodes: Sequence[node], x: int, y: int, distance_func: Callable[[node, node], float] = euclidean):
		# get the neighbours

		criteria = node(x, y)

		nearest = None
		closest_match = float('inf')

		for n in nodes:

			distance = distance_func(n, criteria)
			if distance < closest_match:

				nearest = n
				closest_match = distance

		return nearest

# A* algorithm
class astar(navigator):

	__slots__ = [ 'cost_function', 'heuristic' ]

	def __init__(self, cost: Callable[[Graph, node, node], float], heuristic: Callable[[Graph, node, node], float]):

		self.cost_function = cost
		self.heuristic = heuristic

	def __call__(self, graph: Graph, start: node, goal: node, **kwargs):

		cost_function = self.cost_function
		heuristic = self.heuristic

		# Initialize our visited and to visit lists
		frontier = PriorityQueue()
		frontier.put(start, 0)

		cost_so_far = { start: 0.0 }
		came_from = { start: empty }

		# A* algorithm - core
		while not frontier.empty():

			current = frontier.get()
			if current == goal: break

			previous_cost = cost_so_far[current]
			for neighbor in graph.neighbors(current):

				# Add every single entry, sorting them out by cost - lowest first
				new_cost = previous_cost + cost_function(graph, current, neighbor, **kwargs)
				if neighbor not in came_from or new_cost < cost_so_far[neighbor]:

					cost_so_far[neighbor] = new_cost
					frontier.put(neighbor, new_cost + heuristic(graph, neighbor, goal, **kwargs))
					came_from[neighbor] = current

		# If we found a path, clear it out from the rest - travel backwards to the start, then reverse
		if came_from.get(goal, None):

			output = []
			current = goal
			while current is not empty:

				output.append(current)
				current = came_from[current]

			path = tuple(reversed(output))
			return path

		return ()



# Bellman - Ford Algorithm
class bellman(navigator):

	__slots__ = [ 'cost_function' ]

	def __init__(self, cost: Callable[[Graph, node, node], float]):
		self.cost_function = cost

	def __call__(self, graph: Graph, start: node, goal: node, **kwargs):

		cost_function = self.cost_function

		# Initialize our visited and to visit lists
		frontier = PriorityQueue()
		frontier.put(start, 0)

		total_nodes = len(graph.nodes())
		traversal_depth = { start: 0 }
		cost_so_far = { start: 0.0 }
		came_from = { start: empty }

		while not frontier.empty():

			current = frontier.get()
			if current == goal: break

			previous_cost = cost_so_far[current]
			for neighbor in graph.neighbors(current):

				new_cost = previous_cost + cost_function(graph, current, neighbor, **kwargs)
				depth = traversal_depth.get(current, 0) + 1

				if neighbor not in came_from:

					if depth == total_nodes: raise ValueError

					cost_so_far[neighbor] = new_cost
					frontier.put(neighbor, new_cost)
					came_from[neighbor] = current

				traversal_depth[neighbor] = depth

		# If we found a path, clear it out from the rest - travel backwards to the start, then reverse
		if came_from.get(goal, None):

			output = []
			current = goal
			while current is not empty:

				output.append(current)
				current = came_from[current]

			path = tuple(reversed(output))
			return path

		return {}, ()


# Dijkstra Algorithm
class dijkstra(navigator):

	__slots__ = [ 'cost_function' ]

	def __init__(self, cost: Callable[[Graph, node, node], float]):
		self.cost_function = cost

	def __call__(self, graph: Graph, start: node, goal: node, **kwargs):

		cost_function = self.cost_function

		# Initialize our visited and to visit lists
		frontier = PriorityQueue()
		frontier.put(start, 0)

		cost_so_far = { start: 0.0 }
		came_from = { start: empty }

		# Dijkstra's algorithm - core
		while not frontier.empty():

			current = frontier.get()
			if current == goal: break

			previous_cost = cost_so_far[current]
			for neighbor in graph.neighbors(current):

				# Add every single entry, sorting them out by cost - lowest first
				new_cost = previous_cost + cost_function(graph, current, neighbor, **kwargs)
				if neighbor not in came_from:

					cost_so_far[neighbor] = new_cost
					frontier.put(neighbor, new_cost)
					came_from[neighbor] = current

		# If we found a path, clear it out from the rest - travel backwards to the start, then reverse
		if came_from.get(goal, None):

			output = []
			current = goal
			while current is not empty:

				output.append(current)
				current = came_from[current]

			path = tuple(reversed(output))
			return path

		return (), float('inf')
