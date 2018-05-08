from grid import *
from particle import Particle
from utils import *
from setting import *
import numpy as np
import scipy
from sklearn import mixture

# create a normal distirbutions to calculate a probability within
norm_dist_trans = scipy.stats.norm(0, MARKER_TRANS_SIGMA)
norm_dist_rot = scipy.stats.norm(0, MARKER_ROT_SIGMA)

def motion_update(particles, odom):
	""" Particle filter motion update

		Arguments: 
		particles -- input list of particle represents belief p(x_{t-1} | u_{t-1})
				before motion update
		odom -- odometry to move (dx, dy, dh) in *robot local frame*

		Returns: the list of particles represents belief \tilde{p}(x_{t} | u_{t})
				after motion update
	"""

	motion_particles = []
	for p in particles:
		odom = add_odometry_noise(odom, ODOM_HEAD_SIGMA, ODOM_TRANS_SIGMA)
		dx, dy = rotate_point(odom[0], odom[1], p.h)
		dh = odom[2]
		motion_particles.append(Particle(p.x + dx, p.y + dy, p.h + dh))

	return motion_particles

# ------------------------------------------------------------------------
def measurement_update(particles, measured_marker_list, grid):
	""" Particle filter measurement update

		Arguments: 
			particles -- input list of particle represents belief \tilde{p}(x_{t} | u_{t})
				before meansurement update (but after motion update)

			measured_marker_list -- robot detected marker list, each marker has format:
				measured_marker_list[i] = (rx, ry, rh)
				rx -- marker's relative X coordinate in robot's frame
				ry -- marker's relative Y coordinate in robot's frame
				rh -- marker's relative heading in robot's frame, in degree

				* Note that the robot can only see markers which is in its camera field of view,
				which is defined by ROBOT_CAMERA_FOV_DEG in setting.py
				* Note that the robot can see mutliple markers at once, and may not see any one

			grid -- grid world map, which contains the marker information, 
				see grid.py and CozGrid for definition
				Can be used to evaluate particles

		Returns: the list of particles represents belief p(x_{t} | u_{t})
			after measurement update
	"""

	weights = np.zeros(len(particles))
	for i in range(len(particles)):
		p = particles[i]

		gaussian = 1
		markers = p.read_markers(grid)
		for n in range(len(measured_marker_list)):
			gauss_sum = 0
			for m in range(len(markers)):
				my_marker = markers[m]
				meas_marker = measured_marker_list[n]
				gauss_sum += norm_dist_trans.pdf(my_marker[0] - meas_marker[0]) * norm_dist_trans.pdf(my_marker[1] - meas_marker[1]) * norm_dist_rot.pdf(my_marker[2] - meas_marker[2])
			gaussian *= gauss_sum

		if gaussian == 1:
			gaussian = 0
		weights[i] = gaussian

	total = np.sum(weights)
	if total != 0:
		weights /= np.sum(weights)
		measured_particles = np.random.choice(particles, len(particles), p=weights)
	else:
		measured_particles = particles

	return measured_particles
