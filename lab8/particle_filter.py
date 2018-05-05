from grid import *
from particle import Particle
from utils import *
from setting import *
import numpy as np
import scipy
from sklearn import mixture

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

	# find the actual distance to each marker measured by the robot
	dists = []
	for m in measured_marker_list:
		dists.append(grid_distance(0, 0, m[0], m[1]))
	dists.sort()

	# create a normal distirbution to calculate a probability within
	norm_dist = scipy.stats.norm(0, 5)

	weights = np.zeros(len(particles))
	for i in range(len(particles)):
		p = particles[i]
		
		# get the markers 
		markers = p.read_markers(grid)
		my_dists = []
		for m in markers:
			my_dists.append(grid_distance(0, 0, m[0], m[1]))		
		my_dists.sort()

		if len(dists) > 0 and len(my_dists) > 0:
			weight = norm_dist.pdf(my_dists[0] - dists[0])
			weights[i] = weight

	norm = np.linalg.norm(weights)
	if norm != 0:
		weights /= norm

	measured_particles = particles
#	measured_particles = np.random.choice(particles, len(particles), p=weights)

	return measured_particles

# read_markers, compare sensor markers to particle markers
# use gaussian to determine probability and get weights for each particle
# normalize
# resample to get same number of particles, but using weights distribution.
