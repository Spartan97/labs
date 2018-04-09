#!/usr/bin/env python3

import asyncio
import sys
import time

import cv2
import numpy as np

# I copied find_ball into the lab5 folder so I could tweak the parameters
import find_ball

import cozmo

try:
	from PIL import ImageDraw, ImageFont
except ImportError:
	sys.exit('run `pip3 install --user Pillow numpy` to run this example')


# Define a decorator as a subclass of Annotator; displays battery voltage
class BatteryAnnotator(cozmo.annotate.Annotator):
	def apply(self, image, scale):
		d = ImageDraw.Draw(image)
		bounds = (0, 0, image.width, image.height)
		batt = self.world.robot.battery_voltage
		text = cozmo.annotate.ImageText('BATT %.1fv' % batt, color='green')
		text.render(d, bounds)

# Define a decorator as a subclass of Annotator; displays the ball
class BallAnnotator(cozmo.annotate.Annotator):
	ball = None

	def apply(self, image, scale):
		d = ImageDraw.Draw(image)
		bounds = (0, 0, image.width, image.height)

		if BallAnnotator.ball is not None:

			#double size of bounding box to match size of rendered image
			BallAnnotator.ball = np.multiply(BallAnnotator.ball,2)

			#define and display bounding box with params:
			#msg.img_topLeft_x, msg.img_topLeft_y, msg.img_width, msg.img_height
			box = cozmo.util.ImageBox(BallAnnotator.ball[0]-BallAnnotator.ball[2],
										BallAnnotator.ball[1]-BallAnnotator.ball[2],
										BallAnnotator.ball[2]*2, BallAnnotator.ball[2]*2)
			cozmo.annotate.add_img_box_to_image(image, box, "green", text=None)

			BallAnnotator.ball = None

async def run(robot: cozmo.robot.Robot):
	'''The run method runs once the Cozmo SDK is connected.'''

	#add annotators for battery level and ball bounding box
	robot.world.image_annotator.add_annotator('battery', BatteryAnnotator)
	robot.world.image_annotator.add_annotator('ball', BallAnnotator)

	robot.move_lift(-3)
	robot.set_head_angle(cozmo.util.degrees(-10)).wait_for_completed()

	try:
		while True:
			#get camera image
			event = await robot.world.wait_for(cozmo.camera.EvtNewRawCameraImage, timeout=30)

			#convert camera image to opencv format
			opencv_image = cv2.cvtColor(np.asarray(event.image), cv2.COLOR_RGB2GRAY)
			height, width = opencv_image.shape
			center = width/2.0
			threshold = 60 # threshold for the center of the image

			#find the ball
			ball = find_ball.find_ball(opencv_image, False)
			x = ball[0]
			y = ball[1]
			radius = ball[2]

			#set annotator ball
			BallAnnotator.ball = ball

			motor_left = motor_right = 0

			# spin in a circle until a ball is found
			if x == 0 and y == 0 and radius == 0:
				motor_left = -10
				motor_right = 10
			
			# hit the ball when we're close enough
			elif radius > 115:
				await robot.drive_wheels(0, 0)
				time.sleep(1.0)
				robot.move_lift(1)
				time.sleep(5.0)
				break

			elif abs(x-center) < threshold:
				motor_left = 10
				motor_right = 10

			elif x > center:
				motor_left = 10
			
			elif x < center:
				motor_right = 10

			await robot.drive_wheels(motor_left, motor_right)
			time.sleep(.1)

	except KeyboardInterrupt:
		print("")
		print("Exit requested by user")
	except cozmo.RobotBusy as e:
		print(e)

if __name__ == '__main__':
	cozmo.run_program(run, use_viewer = True, force_viewer_on_top = True)
