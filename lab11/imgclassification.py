#!/usr/bin/env python

##############
#### Your name:
##############

import numpy as np
import re
from sklearn import svm, metrics, feature_extraction, neighbors
from skimage import io, feature, filters, exposure, color
import matplotlib.pyplot as plt
import cozmo
import asyncio
import sys
import time
import cv2

class ImageClassifier:
	def __init__(self):
		self.classifer = None

	def imread_convert(self, f):
		return io.imread(f).astype(np.uint8)

	def load_data_from_folder(self, dir):
		# read all images into an image collection
		ic = io.ImageCollection(dir+"*.bmp", load_func=self.imread_convert)

		#create one large array of image data
		data = io.concatenate_images(ic)

		#extract labels from image names
		labels = np.array(ic.files)
		for i, f in enumerate(labels):
			m = re.search("_", f)
			labels[i] = f[len(dir):m.start()]

		return(data,labels)

	# Preprocess and extract the features of the images
	def extract_image_features(self, data):
		feature_data = []
		feature_images = []

		for image in data:
			# try blurring image
			image = exposure.rescale_intensity(image, "image")
			image = color.rgb2gray(image) # convert to grayscale
			fd = feature.hog(image, orientations=16, pixels_per_cell=(24,24), cells_per_block=(8,8), visualise=False)
			feature_data.append(fd)

		return(feature_data)

	# Train model and save the trained model to self.classifier
	def train_classifier(self, train_data, train_labels):
		self.classifier = neighbors.KNeighborsClassifier(n_neighbors=7, weights="distance")
		self.classifier.fit(train_data, train_labels)

	# predict labels of test data using trained model in self.classifier
	def predict_labels(self, data):
		predicted_labels = self.classifier.predict(data)
		return predicted_labels

def main():
	img_clf = ImageClassifier()

	# load images
	(train_raw, train_labels) = img_clf.load_data_from_folder('./train/')
	(test_raw, test_labels) = img_clf.load_data_from_folder('./test/')

	# convert images into features
	train_data = img_clf.extract_image_features(train_raw)
	test_data = img_clf.extract_image_features(test_raw)

	# train model and test on training data
	img_clf.train_classifier(train_data, train_labels)
	predicted_labels = img_clf.predict_labels(train_data)
	print("\nTraining results")
	print("=============================")
	print("Confusion Matrix:\n",metrics.confusion_matrix(train_labels, predicted_labels))
	print("Accuracy: ", metrics.accuracy_score(train_labels, predicted_labels))
	print("F1 score: ", metrics.f1_score(train_labels, predicted_labels, average='micro'))

	# test model
	predicted_labels = img_clf.predict_labels(test_data)
	print("\nTraining results")
	print("=============================")
	print("Confusion Matrix:\n",metrics.confusion_matrix(test_labels, predicted_labels))
	print("Accuracy: ", metrics.accuracy_score(test_labels, predicted_labels))
	print("F1 score: ", metrics.f1_score(test_labels, predicted_labels, average='micro'))

async def robot_main(robot: cozmo.robot.Robot):
	img_clf = ImageClassifier()
	(train_raw, train_labels) = img_clf.load_data_from_folder('./train/')
	train_data = img_clf.extract_image_features(train_raw)
	img_clf.train_classifier(train_data, train_labels)

	robot.move_lift(-3)
	await robot.set_head_angle(cozmo.util.degrees(-10)).wait_for_completed()
	robot.camera.image_stream_enabled = True
	robot.camera.color_image_enabled = True

	images = []

	while True:
		event = await robot.world.wait_for(cozmo.camera.EvtNewRawCameraImage, timeout=30)

		#convert camera image to opencv format
		image = cv2.cvtColor(np.asarray(event.image), cv2.COLOR_RGB2GRAY)
		data = img_clf.extract_image_features([image])
		labels = img_clf.predict_labels(data)
		images.append(labels[0])
		print(labels[0])

		if len(images) < 3 or images[-1] != images[-2] or images[-2] != images[-3]:
			continue

		if labels[0] == 'none':
			continue

		await robot.say_text(labels[0], use_cozmo_voice=True, in_parallel=True, num_retries=5).wait_for_completed()

		if labels[0] == 'order':
			pass
		elif labels[0] == 'drone':
			pass
		elif labels[0] == 'inspection':
			robot.drive_wheels(25, -25)
			time.sleep(.25)
			robot.drive_wheels(-25, 25)
			time.sleep(.5)
			robot.drive_wheels(25, -25)
			time.sleep(.25)
		elif labels[0] == 'truck':
			robot.drive_wheels(25, 25)
			time.sleep(.5)
			robot.drive_wheels(-25, -25)
			time.sleep(.5)
		elif labels[0] == 'hands':
			pass
		elif labels[0] == 'place':
			robot.move_lift(3)
			time.sleep(.25)
			robot.move_lift(-3)
			time.sleep(.25)
		elif labels[0] == 'plane':
			pass

		time.sleep(.25)

if __name__ == "__main__":
	cozmo.run_program(robot_main)
	main()
