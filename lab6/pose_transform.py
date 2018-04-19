#!/usr/bin/env python3

'''
This is starter code for Lab 6 on Coordinate Frame transforms.

'''

import asyncio
import cozmo
import numpy
from cozmo.util import degrees
import time

def get_relative_pose(object_pose, reference_frame_pose):
	# ####
	# TODO: Implement computation of the relative frame using numpy.
	# Try to derive the equations yourself and verify by looking at
	# the books or slides bfore implementing.
	# ####

	rel_pos = numpy.subtract(object_pose.position, reference_frame_pose.position)
	rel_ang = object_pose.rotation.angle_z - reference_frame_pose.rotation.angle_z

	return cozmo.util.pose_z_angle(rel_pos.x, rel_pos.y, rel_pos.z, rel_ang)

def find_relative_cube_pose(robot: cozmo.robot.Robot):
	'''Looks for a cube while sitting still, prints the pose of the detected cube
	in world coordinate frame and relative to the robot coordinate frame.'''

	robot.move_lift(-3)
	robot.set_head_angle(degrees(0)).wait_for_completed()
	cube = None

	while True:
		try:
			cube = robot.world.wait_for_observed_light_cube(timeout=30)
			if cube:
				print("Robot pose: %s" % robot.pose)
				print("Cube pose: %s" % cube.pose)
				print("Cube pose in the robot coordinate frame: %s" % get_relative_pose(cube.pose, robot.pose))
				print()
		except asyncio.TimeoutError:
			print("Didn't find a cube")


if __name__ == '__main__':

	cozmo.run_program(find_relative_cube_pose)
