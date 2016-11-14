# -*- coding: utf-8 -*-
from graph.generate import create_graph_file
from importlib import import_module
from graph.navigator import *
from graph.nodes import node
from functools import reduce
from graph.metrics import *
from pathlib import Path
import cProfile, random
from typing import *


test_cases = [ (3,3)  ]
path = Path.cwd() / 'build'

# False to run the test for the algorithms and True for the tests of exec time
profile = True

for x, y in test_cases:

	create_graph_file(x, y, path / ('graph_%i_%i.py' % (x, y)))
	module = import_module('build.graph_%i_%i' % (x, y), __package__)

	# Dynamically load the recently-created file
	graph, nodes = getattr(module, 'graph'), getattr(module, 'nodes')

	# Create the navigators
	navigators = [ astar(cost_function, heuristic), dijkstra(cost_function), bellman(cost_function) ]

	# start = navigator.nearby(nodes, random.randint(0, x - 1), random.randint(0, y - 1))
	# goal = navigator.nearby(nodes, random.randint(0, x - 1), random.randint(0, y - 1))

	start = navigator.nearby(nodes, 0, 0)
	goal = navigator.nearby(nodes, x-1, y-1)

	# Run
	if profile:

		print('Profiling path for graph %ix%i...' % (x, y))
		print('Directing from %r to %r: ' % (start, goal))
		print ()

		for nav in navigators:

			print (nav.__class__.__name__.title() + ':')
			cProfile.runctx('nav(graph, start, goal)', globals(), locals())

		del graph, nodes, start, goal
	else:

		print ('Building path for graph %ix%i...' % (x, y))
		print ('Directing from %r to %r: ' % (start, goal))
		print ()

		for nav in navigators:

			costs = {}
			path = nav(graph, start, goal, costs = costs)

			def reduce_costs(value: Tuple[float, float, float], current: node) :

				d1, t1, c1 = value
				d2, t2, c2 = costs.get(current, (0, 0, 0))

				return (d1 + d2, t1 + t2, c1 + c2)

			costs = reduce(reduce_costs, path[:-1], (0, 0, 0))
			print ('%s: path = %r, cost = %r, total_cost = %.2f' % (nav.__class__.__name__, path, costs, sum(costs)))

		print ()
