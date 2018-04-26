#!/usr/bin/env python3

'''
Stater code for Lab 7.

'''

import cozmo
from cozmo.util import degrees, Angle, Pose, distance_mm, speed_mmps
import math
import time
import numpy as np

# Wrappers for existing Cozmo navigation functions

def cozmo_drive_straight(robot, dist, speed):
	"""Drives the robot straight.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		dist -- Desired distance of the movement in millimeters
		speed -- Desired speed of the movement in millimeters per second
	"""
	robot.drive_straight(distance_mm(dist), speed_mmps(speed)).wait_for_completed()

def cozmo_turn_in_place(robot, angle, speed):
	"""Rotates the robot in place.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		angle -- Desired distance of the movement in degrees
		speed -- Desired speed of the movement in degrees per second
	"""
	robot.turn_in_place(degrees(angle), speed=degrees(speed)).wait_for_completed()

def cozmo_go_to_pose(robot, x, y, angle_z):
	"""Moves the robot to a pose relative to its current pose.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		x,y -- Desired position of the robot in millimeters
		angle_z -- Desired rotation of the robot around the vertical axis in degrees
	"""
	robot.go_to_pose(Pose(x, y, 0, angle_z=degrees(angle_z)), relative_to_robot=True).wait_for_completed()

# Functions to be defined as part of the labs

def get_front_wheel_radius():
	"""Returns the radius of the Cozmo robot's front wheel in millimeters."""
	# ####
	# TODO: Empirically determine the radius of the robot's front wheel using the
	# cozmo_drive_straight() function. You can write a separate script for doing 
	# experiments to determine the radius. This function should return the radius
	# in millimeters. Write a comment that explains how you determined it and any
	# computation you do as part of this function.
	# ####

	# to find this, I wrote a test_program and used trial-and-error to find a drive_straight dist argument
	# that resulted in one complete rotation of the front wheel. This value is the circumference of that
	# wheel, from which I calculated the radius by dividing by 2pi.

	return (86/(2*math.pi))

def get_distance_between_wheels():
	"""Returns the distance between the wheels of the Cozmo robot in millimeters."""
	# ####
	# TODO: Empirically determine the distance between the wheels of the robot using
	# robot.drive_wheels() function. Write a comment that explains how you determined
	# it and any computation you do as part of this function.
	# ####

	# to find this, I wrote a test_program2 and drove the left wheel at a speed and timed how long it took
	# to create a full circle. This time and speed could give me the distance of the circumference of a circle 
	# with distance_between_wheels as its radius. It took ~24 seconds at 25mm/s, so my circle would have had a circumference of ~600mm.
	# From that, I can calculate the radius (distance_between_wheels) by dividing by 2pi.

	return (600/(2*math.pi))

def rotate_back_wheel(robot, angle_deg):
	"""Rotates the back wheel of the robot by a desired angle.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		angle_deg -- Desired rotation of the wheel in degrees
	"""

	cozmo_drive_straight(robot, (62*angle_deg)/360, 10)

def my_drive_straight(robot, dist, speed):
	"""Drives the robot straight.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		dist -- Desired distance of the movement in millimeters
		speed -- Desired speed of the movement in millimeters per second
	"""
	
	# subtracting 5 from distance because tihs seems to be slightly different than cozmo_drive_straight
	robot.drive_wheels(speed, speed, duration=(dist-5)/speed)

def my_turn_in_place(robot, angle, speed):
	"""Rotates the robot in place.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		angle -- Desired distance of the movement in degrees
		speed -- Desired speed of the movement in degrees per second
	"""

	circ = math.pi*get_distance_between_wheels()
	robot.drive_wheels(speed, -speed, duration=(circ*angle/360)/speed)

def my_go_to_pose1(robot, x, y, angle_z):
	"""Moves the robot to a pose relative to its current pose.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		x,y -- Desired position of the robot in millimeters
		angle_z -- Desired rotation of the robot around the vertical axis in degrees
	"""
	# ####
	# TODO: Implement a function that makes the robot move to a desired pose
	# using the my_drive_straight and my_turn_in_place functions. This should
	# include a sequence of turning in place, moving straight, and then turning
	# again at the target to get to the desired rotation (Approach 1).
	# ####

	tar = np.array([x, y])
	loc = np.array([robot.pose.position.x, robot.pose.position.y])

	dir = tar - loc

	dir_u = dir / np.linalg.norm(dir)
	loc_u = loc / np.linalg.norm(loc)
	angle_between = np.degrees(np.arccos(np.clip(np.dot(dir_u, loc_u), -1.0, 1.0)))

	print(angle_between, int(angle_between))
	my_turn_in_place(robot, angle_between, 10)

def my_go_to_pose2(robot, x, y, angle_z):
	"""Moves the robot to a pose relative to its current pose.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		x,y -- Desired position of the robot in millimeters
		angle_z -- Desired rotation of the robot around the vertical axis in degrees
	"""
	# ####
	# TODO: Implement a function that makes the robot move to a desired pose
	# using the robot.drive_wheels() function to jointly move and rotate the 
	# robot to reduce distance between current and desired pose (Approach 2).
	# ####
	pass

def my_go_to_pose3(robot, x, y, angle_z):
	"""Moves the robot to a pose relative to its current pose.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		x,y -- Desired position of the robot in millimeters
		angle_z -- Desired rotation of the robot around the vertical axis in degrees
	"""
	# ####
	# TODO: Implement a function that makes the robot move to a desired pose
	# as fast as possible. You can experiment with the built-in Cozmo function
	# (cozmo_go_to_pose() above) to understand its strategy and do the same.
	# ####
	pass

def run(robot: cozmo.robot.Robot):

#	print("***** Back wheel radius: " + str(get_front_wheel_radius()))
#	print("***** Distance between wheels: " + str(get_distance_between_wheels()))

	## Example tests of the functions

#	cozmo_drive_straight(robot, 62, 50)
#	cozmo_turn_in_place(robot, 60, 30)
#	cozmo_go_to_pose(robot, 100, 100, 45)

#	rotate_back_wheel(robot, 90)
#	my_drive_straight(robot, 62, 50)
#	my_turn_in_place(robot, 90, 30)

	my_go_to_pose1(robot, 100, 100, 45)
#	my_go_to_pose2(robot, 100, 100, 45)
#	my_go_to_pose3(robot, 100, 100, 45)


if __name__ == '__main__':

	cozmo.run_program(run)



