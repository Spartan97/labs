
#author1:
#author2:

from grid import *
from visualizer import *
import threading
from queue import PriorityQueue
import math
import cozmo
from collections import defaultdict
import numpy as np

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

scale = 25
def convert_pose_to_coords(pose):
	return (int(pose.position.x/scale + 5), int(pose.position.y/scale + 5))

def expand_obstacle(coord):
	coords = []
	for n in range(-2, 3):
		for m in range(-2, 3):
			coords.append((coord[0] + n, coord[1] + m))

	return coords

def isFound(cube):
	return not (cube.pose.position.x == 0 and cube.pose.position.y == 0 and cube.pose.position.z == 0)

def cozmoBehavior(robot: cozmo.robot.Robot):
	"""Cozmo search behavior. See assignment description for details

		Has global access to grid, a CozGrid instance created by the main thread, and
		stopevent, a threading.Event instance used to signal when the main thread has stopped.
		You can use stopevent.is_set() to check its status or stopevent.wait() to wait for the
		main thread to finish.

		Arguments:
		robot -- cozmo.robot.Robot instance, supplied by cozmo.run_program
	"""

	global grid, stopevent, scale

	robot.set_head_angle(cozmo.util.degrees(0)).wait_for_completed()

	while not stopevent.is_set():
		grid.clearObstacles()
		grid.setStart(convert_pose_to_coords(robot.pose))

		# Find up to three light cubes using built-in functions
		cubes = [robot.world.get_light_cube(cozmo.objects.LightCube1Id), robot.world.get_light_cube(cozmo.objects.LightCube2Id), robot.world.get_light_cube(cozmo.objects.LightCube3Id)]
		desired_angle = cubes[0].pose.rotation.angle_z.degrees

		# Set the location of the goal in the grid, or set it to the middle
		if isFound(cubes[0]):
			grid.clearGoals()
			target_coords = convert_pose_to_coords(cubes[0].pose)
			approach_x = approach_y = 0
			grid.addObstacles(expand_obstacle(target_coords))
			approach_angle = cubes[0].pose.rotation.angle_z
			approach_x = 3*round(math.cos(approach_angle.radians), 0)
			approach_y = 3*round(math.sin(approach_angle.radians), 0)
			grid.addGoal((target_coords[0] - approach_x, target_coords[1] - approach_y))
		else:
			while not isFound(cubes[0]):
				robot.turn_in_place(cozmo.util.degrees(10)).wait_for_completed()
				cubes[0] = robot.world.get_light_cube(cozmo.objects.LightCube1Id)

		if isFound(cubes[1]):
			grid.addObstacles(expand_obstacle(convert_pose_to_coords(cubes[1].pose)))
		if isFound(cubes[2]):
			grid.addObstacles(expand_obstacle(convert_pose_to_coords(cubes[2].pose)))

		# Run Astar
		astar(grid, heuristic)

		# Move to next
		dx = (grid.getPath()[1][0] - grid.getPath()[0][0])*scale
		dy = (grid.getPath()[1][1] - grid.getPath()[0][1])*scale
		dh = cozmo.util.radians(math.atan2(dy, dx)) - robot.pose.rotation.angle_z
		robot.turn_in_place(dh).wait_for_completed()
		robot.drive_straight(cozmo.util.distance_mm(math.sqrt(dx*dx + dy*dy)), cozmo.util.speed_mmps(25)).wait_for_completed()

		if math.sqrt((robot.pose.position.x - cubes[0].pose.position.x)**2 + (robot.pose.position.y - cubes[0].pose.position.y)**2) < 100:
			robot.turn_in_place(cozmo.util.degrees(desired_angle - robot.pose.rotation.angle_z.degrees)).wait_for_completed()
			stopevent.set()

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
