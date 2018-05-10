
#author1:
#author2:

from grid import *
from visualizer import *
import threading
from queue import PriorityQueue
import math
import cozmo
from collections import defaultdict

def find_next_to_eval(open_set, f_score):
	min = open_set[0]
	for node in open_set:
		if f_score[node] < f_score[min]:
			min = node

	return min

def heuristic(current, goal):
	"""Heuristic function for A* algorithm

		Arguments:
		current -- current cell
		goal -- desired goal cell
	"""

	return math.sqrt((goal[0] - current[0])**2 + (goal[1] - current[1])**2)

def astar(grid, heuristic):
	"""Perform the A* search algorithm on a defined grid
	
		Arguments:
		grid -- CozGrid instance to perform search on
		heuristic -- supplied heuristic function
	"""

	start = grid.getStart()
	goal = grid.getGoals()[0]

	# Set of visited nodes
	grid.clearVisited()
	
	# Set of discovered nodes not yet evaluated
	open_set = [start]

	# The most efficient way to each node
	# Will eventually be used to construct the path
	came_from = {}

	# Cheapest cost of getting to each node
	g_score = defaultdict(lambda : 999999)
	g_score[start] = 0
	
	# Cheapest (estimated) cost of reaching the goal while
	# passing through each node
	f_score = defaultdict(lambda : 999999)
	f_score[start] = heuristic(start, goal)

	while len(open_set) != 0:
		current = find_next_to_eval(open_set, f_score)

		# If we're at the goal, reconstruct the path and return it
		if current == goal:
			path = [current]
			while current in came_from:
				path = [came_from[current]] + path
				current = came_from[current]
			grid.setPath(path)
			return

		del open_set[open_set.index(current)]
		grid.addVisited(current)

		for neighbor, weight in grid.getNeighbors(current):
			if neighbor in grid.getVisited():
				continue

			# If we discoverd a new node, add it
			if neighbor not in open_set:
				open_set.append(neighbor)

			temp_score = g_score[current] + weight

			# Skip if this path isn't better than the one we already have
			if temp_score >= g_score[neighbor]:
				continue

			came_from[neighbor] = current
			g_score[neighbor] = temp_score
			f_score[neighbor] = temp_score + heuristic(neighbor, goal)

	print("Failed to find a valid path")
	return

def cozmoBehavior(robot: cozmo.robot.Robot):
	"""Cozmo search behavior. See assignment description for details

		Has global access to grid, a CozGrid instance created by the main thread, and
		stopevent, a threading.Event instance used to signal when the main thread has stopped.
		You can use stopevent.is_set() to check its status or stopevent.wait() to wait for the
		main thread to finish.

		Arguments:
		robot -- cozmo.robot.Robot instance, supplied by cozmo.run_program
	"""

	global grid, stopevent

	while not stopevent.is_set():
		# Find the three light cubes using built-in functions
		cube1 = robot.world.get_light_cube(LightCube1Id).pose  # looks like a paperclip
		cube2 = robot.world.get_light_cube(LightCube2Id).pose  # looks like a lamp / heart
		cube3 = robot.world.get_light_cube(LightCube3Id).pose  # looks like the letters 'ab' over 'T'

		# Set the location of the objects/goal in the grid based on the cubes
		grid.addGoal((cube1.position.x, cube1.position.y))
		grid.addObstacle((cube2.position.x, cube2.position.y))
		grid.addObstacle((cube3.position.x, cube3.position.y))

		# Run Astar
		# Should approach Cube #1 from a specific side without hitting cubes 2 or 3
		pass


######################## DO NOT MODIFY CODE BELOW THIS LINE ####################################


class RobotThread(threading.Thread):
	"""Thread to run cozmo code separate from main thread
	"""
        
	def __init__(self):
		threading.Thread.__init__(self, daemon=True)

	def run(self):
		cozmo.run_program(cozmoBehavior)


# If run as executable, start RobotThread and launch visualizer with empty grid file
if __name__ == "__main__":
	global grid, stopevent
	stopevent = threading.Event()
	grid = CozGrid("emptygrid.json")
	visualizer = Visualizer(grid)
	updater = UpdateThread(visualizer)
	updater.start()
	robot = RobotThread()
	robot.start()
	visualizer.start()
	stopevent.set()
