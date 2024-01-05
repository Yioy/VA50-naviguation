import numpy as np
import cv2 as cv

import fuzzylines
import fish2bird

class TrajectoryExtractor (object):
	"""Main class for the ROS node that manages the trajectory"""

	#                        ╔══════════════════════╗                       #
	# ═══════════════════════╣    INITIALISATION    ╠══════════════════════ #
	#                        ╚══════════════════════╝                       #

	def __init__(self,
			  parameters,
			  camera_to_image,
			  distortion_parameters
			  ):
		"""Initialize the node and everything that it needs
		   - parameters   : dict<str: …>        : Node parameters, from the parameter file
		"""
		self.parameters = parameters

		self.camera_to_image = camera_to_image
		self.distortion_parameters = distortion_parameters
		
		# Initialize the fuzzy systems
		self.init_fuzzysystems()

		# Bird-eye projection parameters, for convenience
		self.birdeye_range_x = (-self.parameters["birdeye"]["x-range"], self.parameters["birdeye"]["x-range"])
		self.birdeye_range_y = (self.parameters["birdeye"]["roi-y"], self.parameters["birdeye"]["y-range"])
	
	
	def init_fuzzysystems(self):
		"""Initialize the fuzzy systems used by the lane detection"""
		line_variables = ("forward-distance", "line-distance", "line-lengths", "parallel-distances", "parallel-angles")
		line_centers = np.asarray([self.parameters["fuzzy-lines"]["centers"][variable] for variable in line_variables])
		line_malus = np.asarray([self.parameters["fuzzy-lines"]["malus"][variable] for variable in line_variables], dtype=int)
		line_output_centers = np.asarray(self.parameters["fuzzy-lines"]["centers"]["output"])
		self.lane_system = fuzzylines.FuzzySystem(line_centers, line_malus, line_output_centers, self.parameters["fuzzy-lines"]["base-score"])
		

	def preprocess_image(self, image, target_to_camera):
		"""Preprocess the image receive from the camera
		   - image            : ndarray[y, x, 3] : RGB image received from the camera
		   - target_to_camera : ndarray[4, 4]    : 3D homogeneous transform matrix from the target (road) frame to the camera frame
		<---------------------- ndarray[v, u]    : Full grayscale bird-eye view (mostly for visualisation)
		<---------------------- ndarray[v, u]    : Fully preprocessed bird-eye view (binarized, edge-detected)
		<---------------------- float            : Scale factor, multiply by this to convert lengths from pixel to metric in the target frame
		"""
		# Convert the image to grayscale
		grayimage = cv.cvtColor(image, cv.COLOR_RGB2GRAY)

		# Binarize the image. First a gaussian blur is applied to reduce noise,
		img_blur = cv.GaussianBlur(grayimage, (7, 7), 1.5)
		
		# Project in bird-eye view
		# then a gaussian adaptive thresholding is applied to reduce the influence of lighting changes
		birdeye, scale_factor = fish2bird.to_birdeye(img_blur, self.camera_to_image, target_to_camera, self.distortion_parameters[0], self.birdeye_range_x, self.birdeye_range_y, self.parameters["birdeye"]["birdeye-size"], interpolate=True, flip_y=True)
		be_binary = cv.adaptiveThreshold(birdeye, 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C, cv.THRESH_BINARY, self.parameters["preprocess"]["threshold-window"], self.parameters["preprocess"]["threshold-bias"])

		# The adaptive binarization makes the borders white, mask them out
		mask = cv.erode(np.uint8(birdeye > 0), cv.getStructuringElement(cv.MORPH_RECT, (self.parameters["preprocess"]["threshold-window"]//2 + 2, self.parameters["preprocess"]["threshold-window"]//2 + 2)))
		be_binary *= mask

		# Apply an opening operation to eliminate a few artifacts and better separate blurry markings
		open_kernel_size = self.parameters["preprocess"]["open-kernel-size"]
		open_kernel = cv.getStructuringElement(cv.MORPH_RECT, (open_kernel_size, open_kernel_size))
		be_binary = cv.morphologyEx(be_binary, cv.MORPH_OPEN, open_kernel)

		# Edge detection to get the 1-pixel wide continuous curves required by the following operations
		be_binary = cv.Canny(be_binary, 50, 100)
		return birdeye, be_binary, scale_factor
	
	#                      ╔═════════════════════════╗                      #
	# ═════════════════════╣ BIRD-EYE VIEW UTILITIES ╠═════════════════════ #
	#                      ╚═════════════════════════╝                      #

	# Those are just wrappers to the fish2bird functions, to use the global parameters
	def target_to_birdeye(self, be_binary, target_points):
		return fish2bird.target_to_output(target_points, self.birdeye_range_x, self.birdeye_range_y, be_binary.shape[0], flip_y=True)[0]

	def birdeye_to_target(self, be_binary, image_points):
		return fish2bird.birdeye_to_target(image_points, self.birdeye_range_x, self.birdeye_range_y, be_binary.shape, flip_y=True)[:2]
	

