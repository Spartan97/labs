#!/usr/bin/env python3

'''
This is starter code for Lab 7.

'''

import cozmo
from cozmo.util import degrees, Angle, Pose, distance_mm, speed_mmps
import math
import time
import sys

import odometry as odo
sys.path.insert(0, '../lab6')
from pose_transform import get_relative_pose

def move_relative_to_cube(robot: cozmo.robot.Robot):
	'''Looks for a cube while sitting still, when a cube is detected it 
	moves the robot to a given pose relative to the detected cube pose.'''

	robot.move_lift(-3)
	robot.set_head_angle(degrees(0)).wait_for_completed()
	cube = None

	while cube is None:
		try:
			cube = robot.world.wait_for_observed_light_cube(timeout=30)
			if cube:
				print("Found a cube, pose in the robot coordinate frame: %s" % get_relative_pose(cube.pose, robot.pose))
		except asyncio.TimeoutError:
			print("Didn't find a cube")

	desired_pose_relative_to_cube = Pose(0, 100, 0, angle_z=degrees(90))

	#### Approach 1 -- turn to face cube, then move to cube, then turn in place to heading
	desired_pose = get_relative_pos(desired_pose_relative_to_cube, cube.pose)
	robot_pose = robot.pose

def test_program(robot: cozmo.robot.Robot):
	print(odo.get_front_wheel_radius())
	odo.cozmo_drive_straight(robot, 86, 10)

def test_program2(robot: cozmo.robot.Robot):
	print(odo.get_distance_between_wheels())
	while True:
		robot.drive_wheels(25, 0)

def test_program3(robot: cozmo.robot.Robot):
	odo.rotate_back_wheel(robot, 180)

def test_program4(robot: cozmo.robot.Robot):
	odo.my_drive_straight(robot, 86, 10)

def test_program5(robot: cozmo.robot.Robot):
	odo.my_turn_in_place(robot, 360, 25)

if __name__ == '__main__':
#	cozmo.run_program(test_program5)
	cozmo.run_program(move_relative_to_cube)
