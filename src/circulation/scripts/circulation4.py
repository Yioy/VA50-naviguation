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

from circulation.msg import Trajectory
from trafficsigns.msg import TrafficSignStatus, TrafficSign


# TODO organiser
import trajectory_extractor.circulation_enums as enums
from trajectory_extractor.IntersectionHint import IntersectionHint
from trajectory_extractor.TrajectoryVisualizer import TrajectoryVisualizer
from trajectory_extractor.TrajectoryExtractor import TrajectoryExtractor

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


		
		# get camera_info
		self.camera_info_msg = None
		self.camerainfo_subscriber = rospy.Subscriber(self.parameters["node"]["camerainfo-topic"], CameraInfo, self.callback_camerainfo, queue_size=1)
		while not rospy.is_shutdown() and self.camera_info_msg is None:
			time.sleep(0.1)
			rospy.loginfo_once("Waiting for camerainfo...")
		rospy.loginfo("Got camerainfo!")

		# get camera_to_image and distortion_parameters
		camera_to_image = np.asarray(self.camera_info_msg.P).reshape((3, 4))[:, :3]
		distortion_parameters = self.camera_info_msg.D


		# Initialize the transformation listener
		self.tf_buffer = tf2_ros.Buffer(rospy.Duration(120))
		self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
		
		# Get the transform from the camera to the local vehicle frame (base_link)
		self.target_to_camera = None
		self.get_transform(self.parameters["node"]["road-frame"], self.camera_info_msg.header.frame_id)
		while not rospy.is_shutdown() and self.target_to_camera is None:
			self.target_to_camera = self.get_transform(self.parameters["node"]["road-frame"], self.camera_info_msg.header.frame_id)
			time.sleep(0.1)
			rospy.loginfo_once("Waiting for transform from the camera to the vehicle frame...")


		self.trajectory_extractor = TrajectoryExtractor(
			self.parameters,
			camera_to_image,
			distortion_parameters
			)








		# Initialize the topic subscribers (last to avoid too early messages while other things are not yet initialized)
		self.image_subscriber = rospy.Subscriber(self.parameters["node"]["image-topic"], Image, self.callback_image, queue_size=1, buff_size=2**28)
		self.camerainfo_subscriber = rospy.Subscriber(self.parameters["node"]["camerainfo-topic"], CameraInfo, self.callback_camerainfo, queue_size=1)
		self.direction_subscriber = rospy.Subscriber(self.parameters["node"]["direction-topic"], UInt8, self.callback_direction)
		self.trafficsign_subscriber = rospy.Subscriber(self.parameters["node"]["traffic-sign-topic"], TrafficSignStatus, self.callback_trafficsign)
		self.trajectory_publisher = rospy.Publisher(self.parameters["node"]["trajectory-topic"], Trajectory, queue_size=10)
		self.trajectory_seq = 0  # Sequential number of published trajectories

		rospy.loginfo("Ready")

	#                        ╔══════════════════════╗                       #
	# ═══════════════════════╣ SUBSCRIBER CALLBACKS ╠══════════════════════ #
	#                        ╚══════════════════════╝                       #

	def callback_image(self, message):
		"""Callback called when an image is published from the camera
		   - message : sensor_msgs.msg.Image : Message from the camera
		"""
		
		# Extract the image and the timestamp at which it was taken, critical for synchronisation
		rospy.logdebug("------ Received an image")

		image = np.frombuffer(message.data, dtype=np.uint8).reshape((message.height, message.width, 3))
		self.trajectory_extractor.compute_trajectory(image, message.header.stamp, self.target_to_camera)
		#cProfile.runctx("self.compute_trajectory(image, message.header.stamp, message.header.frame_id)", globals(), locals())

	def callback_camerainfo(self, message):
		"""Callback called when a new camera info message is published
		   - message : sensor_msgs.msg.CameraInfo : Message with metadata about the camera
		"""

		# fish2bird only supports the camera model defined by Christopher Mei
		if message.distortion_model.lower() != "mei":
			rospy.logerr(f"Bad distortion model : {message.distortion_model}")
			return
		
		# disable callback after first call
		self.camerainfo_subscriber.unregister()
		
		self.camera_info_msg = message
		
		rospy.loginfo("Got camerainfo!")
	
	def callback_direction(self, message):
		"""Callback called when a direction is sent from the navigation nodes
		   - message : std_msgs.msg.Uint8 : Message with the direction (same values as enums.Direction.FORWARD, .LEFT and .RIGHT)
		"""
		if message.data == enums.Direction.FORWARD:
			rospy.loginfo("Updated next direction to FORWARD")
			self.visualisation.print_message("Manually set next direction : FORWARD")
		elif message.data == enums.Direction.LEFT:
			rospy.loginfo("Updated next direction to LEFT")
			self.visualisation.print_message("Manually set next direction : LEFT")
		elif message.data == enums.Direction.RIGHT:
			rospy.loginfo("Updated next direction to RIGHT")
			self.visualisation.print_message("Manually set next direction : RIGHT")
		elif message.data == enums.Direction.DOUBLE_LANE:
			rospy.logdebug("Next intersection has a double lane")
			self.next_double_lane = True
			return
		elif message.data == enums.Direction.FORCE_INTERSECTION:
			self.visualisation.print_message("Manually force intersection mode")
			if self.next_direction == enums.Direction.LEFT:
				self.switch_intersection(enums.NavigationMode.INTERSECTION_LEFT, rospy.get_rostime(), 0)
			elif self.next_direction == enums.Direction.RIGHT:
				self.switch_intersection(enums.NavigationMode.INTERSECTION_RIGHT, rospy.get_rostime(), 0)
			elif self.next_direction == enums.Direction.FORWARD:
				self.switch_intersection(enums.NavigationMode.INTERSECTION_FORWARD, rospy.get_rostime(), 0)
			return
		else:
			rospy.logerr(f"Invalid direction ID received : {message.data}")
			return
		self.next_direction = message.data
	
	def callback_trafficsign(self, message):
		"""Callback called when traffic signs are detected and received
		   - message : trafficsigns.msg.TrafficSignStatus : Message with the detected traffic signs data
		"""
		for trafficsign in message.traffic_signs:
			if trafficsign.type in enums.TURN_SIGNS and trafficsign.confidence > 0.6:
				rospy.logdebug(f"New traffic sign : {trafficsign.type}, position at {message.header.stamp} [{trafficsign.x}, {trafficsign.y}, {trafficsign.z}], confidence {trafficsign.confidence}")
				self.add_intersection_hint(IntersectionHint("trafficsign", trafficsign.type, (trafficsign.x, trafficsign.y, trafficsign.z), message.header.stamp, trafficsign.confidence))

	#               ╔═══════════════════════════════════════╗               #
	# ══════════════╣ TRANSFORM MANAGEMENT AND MEASUREMENTS ╠══════════════ #
	#               ╚═══════════════════════════════════════╝               #

	def get_transform(self, source_frame, target_frame):
		"""Get the latest transform matrix from `source_frame` to `target_frame`
		   - source_frame : str           : Name of the source frame
		   - target_frame : str           : Name of the target frame
		<---------------- : ndarray[4, 4] : 3D homogeneous transform matrix to convert from `source_frame` to `target_frame`,
		                                    or None if no TF for those frames was published
		"""
		try:
			transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time(0))
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
			return None

		# Build the matrix elements from the translation vector and the rotation quaternion
		rotation_message = transform.transform.rotation
		rotation_quaternion = np.asarray((rotation_message.w, rotation_message.x, rotation_message.y, rotation_message.z))
		rotation_matrix = quaternions.quat2mat(rotation_quaternion)
		translation_message = transform.transform.translation
		translation_vector = np.asarray((translation_message.x, translation_message.y, translation_message.z)).reshape(3, 1)

		# Build the complete transform matrix
		return np.concatenate((
			np.concatenate((rotation_matrix, translation_vector), axis=1),
			np.asarray((0, 0, 0, 1)).reshape((1, 4))
		), axis=0)
	

	
	#                         ╔══════════════════╗                          #
	# ════════════════════════╣ IMAGE PROCESSING ╠═════════════════════════ #
	#                         ╚══════════════════╝                          #


	#                           ╔═══════════════╗                           #
	# ══════════════════════════╣ VISUALISATION ╠══════════════════════════ #
	#                           ╚═══════════════╝                           #

	def viz_intersection_mode(self, viz, scale_factor, image_timestamp, remaining_distance):
		"""Visualization in intersection navigation mode
		   - viz                : ndarray[y, x] : Bird-eye view visualization image
		   - scale_factor       : float         : Scale factor from pixel to metric lengths
		   - image_timestamp    : rospy.Time    : Timestamp to visualize at
		   - remaining_distance : float         : Distance until reaching the rejoin distance
		"""
		transforms, distances = self.get_map_transforms([self.current_trajectory_timestamp], image_timestamp)
		local_trajectory = (transforms[0] @ np.vstack((self.current_trajectory, np.zeros((1, self.current_trajectory.shape[1])), np.ones((1, self.current_trajectory.shape[1])))))[:2]
		viz_trajectory = self.trajectory_extractor.target_to_birdeye(viz, local_trajectory)
		cv.polylines(viz, [viz_trajectory.transpose().astype(int)], False, (60, 255, 255), 2)
		if remaining_distance is not None:
			cv.circle(viz, (viz.shape[1]//2, viz.shape[0]), int(remaining_distance / scale_factor), (0, 255, 255), 1)
	
	def publish_trajectory(self, trajectory_points, trajectory_timestamp):
		"""Publish a trajectory on the output topic
		   - trajectory_points    : ndarray[N, 2] : Points of the trajectory as LINE VECTORS, contrary to the rest of the node
		   - trajectory_timestamp : rospy.Time    : Timestamp at which the trajectory is valid
		"""
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
		
