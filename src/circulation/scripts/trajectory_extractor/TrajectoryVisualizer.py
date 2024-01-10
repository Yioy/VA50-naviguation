import time
import cv2 as cv
import numpy as np

class TrajectoryVisualizer (object):
	"""Quick-and-dirty visualization window management
	   There are 2 visualizations, just merge them into one and call cv.imshow"""
	
	def __init__(self, parameters):
		self.line_viz = None
		self.trajectory_viz = None
		self.message = None
		self.message_time = None

	def update_line_detection(self, be_binary, lines, left_line_index, right_line_index, markings):
		"""Generate and update the left visualization from the preprocessed image and the detected lines and markings
		   - be_binary        : ndarray[y, x]       : Preprocessed camera image (binary edge-detected bird-eye view)
		   - lines            : list<ndarray[2, N]> : Detected discrete curves in the image
		   - left_line_index  : int                 : Index of the left lane marking in the `lines` list, or None
		   - right_line_index : int                 : Index of the right lane marking in the `lines` list, or None
		   - markings         : dict<str, …>        : Dictionary of detected road markings. Currently only supports `crosswalks`
		"""
		self.line_viz = cv.merge((be_binary, be_binary, be_binary))
		for line in lines:
			cv.polylines(self.line_viz, [line.astype(int).transpose()], False, (0, 200, 0), 2)
		if left_line_index is not None:
			cv.polylines(self.line_viz, [lines[left_line_index].astype(int).transpose()], False, (255, 0, 0), 4)
		if right_line_index is not None:
			cv.polylines(self.line_viz, [lines[right_line_index].astype(int).transpose()], False, (0, 100, 255), 4)

		for i, crosswalk in enumerate(markings["crosswalks"]):
			color = ((int(i * 255 / len(markings["crosswalks"])) + 30) % 255, 255, 255)
			for rectangle in crosswalk:
				cv.fillPoly(self.line_viz, [rectangle.astype(int).transpose()], color)


	def update_trajectory_construction(self, viz):
		"""Update the right visualization with the given image"""
		self.trajectory_viz = viz
	
	def print_message(self, message):
		self.message = message
		self.message_time = time.time()

	def show(self):
		"""Update the visualization window"""
		if self.line_viz is None or self.trajectory_viz is None:
			return
		# Just merge both images
		# full_viz = cv.cvtColor(np.concatenate((self.line_viz, np.zeros((self.line_viz.shape[0], 30, 3), dtype=np.uint8), self.trajectory_viz), axis=1), cv.COLOR_RGB2BGR)
		full_viz = cv.cvtColor(np.concatenate((self.line_viz, self.trajectory_viz), axis=1), cv.COLOR_RGB2BGR)
		if self.message is not None:
			if time.time() > self.message_time + 5:
				self.message = None
				self.message_time = None
			else:
				cv.putText(full_viz, self.message, (5, 40), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
		
		# Uncomment these if your visualization window often stays tiny
		## cv.namedWindow("viz", cv.WINDOW_NORMAL)
		## cv.resizeWindow("viz", full_viz.shape[1], full_viz.shape[0])
		cv.imshow("Trajectoire", full_viz)
		cv.waitKey(1)