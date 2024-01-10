#!/usr/bin/env python3

#   Copyright 2023 Grégori MIGNEROT, Élian BELMONTE, Benjamin STACH
#
#   Licensed under the Apache License, Version 2.0 (the "License");
#   you may not use this file except in compliance with the License.
#   You may obtain a copy of the License at
#
#       http://www.apache.org/licenses/LICENSE-2.0
#
#   Unless required by applicable law or agreed to in writing, software
#   distributed under the License is distributed on an "AS IS" BASIS,
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#   See the License for the specific language governing permissions and
#   limitations under the License.

"""
Main module of the ROS node that builds the vehicle’s trajectory

NOTES :
    - There is no static reference frame allowed here, so our reference frame is the vehicle (base_link)
	  As such, it is very important to keep track of the time at which each measurement is taken, whatever it is,
	  so that we know how to pull it back to the local frame at the relevant moment
	- All discrete curves in this node and its submodules are using COLUMN VECTORS (numpy shape [2, N])

TROUBLESHOOTING :
    - If the visualization window often stays ridiculously small, uncomment the marked lines in the method `TrajectoryVisualizer.update`
"""

# ═══════════════════════════ BUILT-IN IMPORTS ════════════════════════════ #
import sys
import time
import itertools
import cProfile

from collections import Counter

# ══════════════════════════ THIRD-PARTY IMPORTS ══════════════════════════ #
import yaml
import cv2 as cv
import numpy as np
import transforms3d.quaternions as quaternions
import transforms3d.euler

# ══════════════════════════════ ROS IMPORTS ══════════════════════════════ #
import rospy
import tf2_ros
from std_msgs.msg import UInt8, Float64MultiArray, MultiArrayDimension, Header
from sensor_msgs.msg import Image, CameraInfo
import message_filters
from cv_bridge import CvBridge

from circulation.msg import Trajectory
from trafficsigns.msg import TrafficSignStatus, TrafficSign


# TODO organiser
import trajectory_extractor.circulation_enums as enums
from trajectory_extractor.IntersectionHint import IntersectionHint
from trajectory_extractor.TrajectoryVisualizer import TrajectoryVisualizer
from trajectory_extractor.TrajectoryExtractor import TrajectoryExtractor
from trajectory_extractor.MultiCamBirdViewRosInitializer import MultiCamBirdViewRosInitializer

# TODO : More resilient lane detection
# TODO : Autonomous intersection detection
# TODO : No direction panic mode
# TODO : Ensure a better base point continuity in trajectories




class TrajectoryExtractorNode (object):
	"""Main class for the ROS node that manages the trajectory"""

	#                        ╔══════════════════════╗                       #
	# ═══════════════════════╣    INITIALISATION    ╠══════════════════════ #
	#                        ╚══════════════════════╝                       #

	def __init__(self, parameters):
		"""Initialize the node and everything that it needs
		   - parameters   : dict<str: …>        : Node parameters, from the parameter file
		"""
		self.parameters = parameters


		# initialize multicam
		image_topics = []
		camera_info_topics = []
		if self.parameters["birdeye"]["multicam"]:
			image_topics = [self.parameters["node"]["image-topic"], self.parameters["node"]["image-topic-left"], self.parameters["node"]["image-topic-right"]]
			camera_info_topics = [self.parameters["node"]["camerainfo-topic"], self.parameters["node"]["camerainfo-topic-left"], self.parameters["node"]["camerainfo-topic-right"]]
		else:
			image_topics = [self.parameters["node"]["image-topic"]]
			camera_info_topics = [self.parameters["node"]["camerainfo-topic"]]

		x_range = (self.parameters["birdeye"]["x-range"][0], self.parameters["birdeye"]["x-range"][1])
		y_range = (self.parameters["birdeye"]["y-range"][0], self.parameters["birdeye"]["y-range"][1])


		ros_multi_cam_birdview_initializer = MultiCamBirdViewRosInitializer()
		multi_cam_birdview_init_config =ros_multi_cam_birdview_initializer.create_init_config(
			camera_info_topics,
			self.parameters["node"]["road-frame"],
			x_range,
			y_range,
			self.parameters["birdeye"]["birdeye-size"],
			flip_x=False,
			flip_y=True)

		# Initialize the trajectory extractor
		self.trajectory_extractor = TrajectoryExtractor(
			self.parameters,
			multi_cam_birdview_init_config,
			)


		# Initialize the topic subscribers (last to avoid too early messages while other things are not yet initialized)
		self.direction_subscriber = rospy.Subscriber(self.parameters["node"]["direction-topic"], UInt8, self.callback_direction)
		self.trafficsign_subscriber = rospy.Subscriber(self.parameters["node"]["traffic-sign-topic"], TrafficSignStatus, self.callback_trafficsign)
		self.trajectory_publisher = rospy.Publisher(self.parameters["node"]["trajectory-topic"], Trajectory, queue_size=10)
		self.trajectory_seq = 0  # Sequential number of published trajectories

		self.bridge = CvBridge()

		if not self.parameters["birdeye"]["multicam"]:
					self.image_subscriber = rospy.Subscriber(self.parameters["node"]["image-topic"], Image, self.single_camera_callback, queue_size=1, buff_size=2**28)
		else:
			self.camera_subscribers = [message_filters.Subscriber(topic, Image) for topic in image_topics]
			self.time_sync = message_filters.ApproximateTimeSynchronizer(self.camera_subscribers, 10, 0.1, allow_headerless=False)
			self.time_sync.registerCallback(self.multi_camera_callback)


		rospy.loginfo("Ready")

	#                        ╔══════════════════════╗                       #
	# ═══════════════════════╣ SUBSCRIBER CALLBACKS ╠══════════════════════ #
	#                        ╚══════════════════════╝                       #

	def multi_camera_callback(self, *args):
		"""Callback for camera topics
		:param args: list of camera images
		"""

		# Convert camera images to cv images
		images = []
		try:
			images = [self.bridge.imgmsg_to_cv2(image, desired_encoding="mono8") for image in args]
		except Exception as e:
			rospy.logerr(f"Error converting image: {e}")
			return
		# Process the images
		self.trajectory_extractor.compute_trajectory(images, args[0].header.stamp)
		trajectory, timestamp = self.trajectory_extractor.get_current_trajectory()
		self.publish_trajectory(trajectory, timestamp)

	def single_camera_callback(self, message):
		"""Callback called when an image is published from the camera
		   - message : sensor_msgs.msg.Image : Message from the camera
		"""
		
		# Extract the image and the timestamp at which it was taken, critical for synchronisation
		rospy.logdebug("------ Received an image")

		image = self.bridge.imgmsg_to_cv2(message, desired_encoding="mono8")
		self.trajectory_extractor.compute_trajectory([image], message.header.stamp)
		trajectory, timestamp = self.trajectory_extractor.get_current_trajectory()
		self.publish_trajectory(trajectory, timestamp)
		#cProfile.runctx("self.compute_trajectory(image, message.header.stamp, message.header.frame_id)", globals(), locals())
	
	def callback_direction(self, message):
		"""Callback called when a direction is sent from the navigation nodes
		   - message : std_msgs.msg.Uint8 : Message with the direction (same values as enums.Direction.FORWARD, .LEFT and .RIGHT)
		"""
		self.trajectory_extractor.set_next_direction(message.data)
	
	def callback_trafficsign(self, message):
		"""Callback called when traffic signs are detected and received
		   - message : trafficsigns.msg.TrafficSignStatus : Message with the detected traffic signs data
		"""
		for trafficsign in message.traffic_signs:
			if trafficsign.type in enums.TURN_SIGNS and trafficsign.confidence > 0.6:
				rospy.logdebug(f"New traffic sign : {trafficsign.type}, position at {message.header.stamp} [{trafficsign.x}, {trafficsign.y}, {trafficsign.z}], confidence {trafficsign.confidence}")
				self.trajectory_extractor.add_intersection_hint(IntersectionHint("trafficsign", trafficsign.type, (trafficsign.x, trafficsign.y, trafficsign.z), message.header.stamp, trafficsign.confidence))
	
	def publish_trajectory(self, trajectory_points, trajectory_timestamp):
		"""Publish a trajectory on the output topic
		   - trajectory_points    : ndarray[N, 2] : Points of the trajectory as LINE VECTORS, contrary to the rest of the node
		   - trajectory_timestamp : rospy.Time    : Timestamp at which the trajectory is valid
		"""
		if trajectory_points is None or trajectory_timestamp is None:
			rospy.logwarn("No trajectory to publish")
			return
		# Add the current position (0, 0) to the trajectory, otherwise the first point might be too far away
		# and the pure pursuit will miss it
		while trajectory_points.shape[0] > 0 and abs(np.pi / 2 - np.arctan2(trajectory_points[0, 1], trajectory_points[0, 0])) > self.parameters["trajectory"]["max-output-angle"]:
			trajectory_points = trajectory_points[1:]
		if trajectory_points.shape[0] == 0:
			rospy.logerr("No remaining points after max output angle cut")
			return
		
		trajectory_points = np.concatenate((np.asarray([[0, 0]]), trajectory_points), axis=0)

		trajectory_array = Float64MultiArray()
		trajectory_array.data = trajectory_points.flatten()
		trajectory_array.layout.data_offset =  0
		dim = []
		dim.append(MultiArrayDimension("points", trajectory_points.shape[0], trajectory_points.shape[0]*trajectory_points.shape[1]))
		dim.append(MultiArrayDimension("coords", trajectory_points.shape[1], trajectory_points.shape[1]))
		trajectory_array.layout.dim = dim

		message = Trajectory()
		message.header = Header(seq=self.trajectory_seq, stamp=trajectory_timestamp, frame_id=self.parameters["node"]["road-frame"])
		message.trajectory = trajectory_array
		self.trajectory_seq += 1

		rospy.logdebug("Publishing a new trajectory")
		self.trajectory_publisher.publish(message)


#                          ╔═════════════════════╗                          #
# ═════════════════════════╣ NODE INITIALISATION ╠═════════════════════════ #
#                          ╚═════════════════════╝                          #

if __name__ == "__main__":
	# I’m fed up with scientific notation
	np.set_printoptions(threshold = sys.maxsize, suppress=True)

	# get filename as ros parameter (if present)
	paramfile_path = rospy.get_param("config_file", default=None)
	
	if paramfile_path is not None:

		rospy.loginfo("Param file path retrieved from ROS param: " + paramfile_path)

		# Load the parameters and map
		with open(paramfile_path, "r") as parameterfile:
			parameters = yaml.load(parameterfile, yaml.Loader)

		# Initialize and start the node
		rospy.init_node(parameters["node"]["trajectory-node-name"])
		node = TrajectoryExtractorNode(parameters)
		rospy.spin()
	else:
		rospy.logfatal("Required ROS param config_file not provided.")
		
