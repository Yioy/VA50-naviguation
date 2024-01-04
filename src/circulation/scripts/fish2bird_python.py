"""
Retrieve the actual 3D position of points from an image
This assumes an omnidirection distortion model as described in the following paper by Christopher Mei :

https://www-sop.inria.fr/icare/personnel/Christopher.Mei/articles/projection_model.pdf

Frame and coordinate systems reference :
 - image  : (u, v)       : input image pixel coordinates
 - input  : (x, y)       : metric plane associated to the image ("sensor frame")
 - sphere : (x, y, z)    : projection on the surface of a unit sphere (Mei distortion model)
 - angles : (θ, φ)       : spheric coordinates angles of the 3D point in the camera frame relative to the center of the unit sphere (θ = colatitude, φ = longitude)
 - camera : (X, Y, Z, 1) : camera frame associated to the input image
 - target : (X, Y, Z, 1) : target plane frame (all target points are assumed to be on a plane where Z=0 in the target frame)
 - output : (u, v)       : output image frame (orthogonal projection along the z axis and scaling)

This module helps retrieving original 3D points from an image.
The original 3D points are assumed to be on a 3D plane
To do that, this modules needs three main parameters :
- camera_to_image  : The K matrix, that projects from metric coordinates to pixel coordinates
                     |αᵤ  0 u₀|
                     | 0 αᵥ v₀|
                     | 0  0  1|
                     camera_to_image @ input_points = image_points
- xi               : The ξ parameter in Christopher Mei’s paper, that gives the displacement from the unit sphere center for the reprojection
- camera_to_target : A full, homogeneous 4×4 transform matrix, that converts from the camera frame to another 3D frame,
                     where all relevant points in the image are assumed to be on the (X, Y) plane (such that Z = 0)
                     For instance, for the forward camera of a vehicle, this may convert from the camera frame
                     to a frame with its origin point on the road and the Z axis pointing upward,
                     all points are then assumed to be on the road (Z = 0 in the road frame)
                     |\ | / Tx|
                     |- R - Ty|
                     |/ | \ Tz|
                     |0 0 0  1|
                     camera_to_target @ camera_point = target_points, target_points[2] == 0
 """

import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
import time


class Fish2Bird:

	def __init__(self, input_image_shape, camera_to_image, camera_to_target, xi, x_range, y_range, output_size_y, flip_x=False, flip_y=False) -> None:
		self.input_image_shape = input_image_shape
		self.camera_to_image = camera_to_image
		self.camera_to_target = camera_to_target
		self.xi = xi
		self.x_range = x_range
		self.y_range = y_range
		self.output_size_y = output_size_y
		self.flip_x = flip_x
		self.flip_y = flip_y

		self.output_pixels = None
		self.output_shape = None
		self.scale_factor = None
		self.output_filter = None
		self.image_points = None

		self.init_projection()


	def get_output_shape(self):
		"""Get the shape of the resulting bird-eye view image"""
		return self.output_shape



	def get_image_points(self, image_shape):
		"""Get the [x, y] coordinates of all pixels of the image
		- image : ndarray[y, x] : source image
		<----------- ndarray[2, N] : Coordinates of all image points as columns (x, y)"""
		y_positions = np.arange(0, image_shape[0], 1)
		x_positions = np.arange(0, image_shape[1], 1)
		x_2d, y_2d = np.meshgrid(x_positions, y_positions)
		x_2d = x_2d.ravel()
		y_2d = y_2d.ravel()
		image_points = np.asarray((x_2d, y_2d))
		return image_points

	def image_to_input(self, image_points, camera_to_image):
		"""Convert from image pixel coordinates to input frame ("sensor") homogeneous coordinates
		- image_points    : ndarray[2, N] : Pixel coordinates in the image (vectors as columns)
		- camera_to_image : ndarray[3, 3] : Projection matrix (K) to pixel coordinates
		<--------------------- ndarray[2, N] : Coordinates in the metric plane associated to the image (input frame)"""
		image_homogeneous = np.asarray((image_points[0], image_points[1], np.ones(image_points.shape[1])))
		return (np.linalg.inv(camera_to_image) @ image_homogeneous)[:2]

	def input_to_sphere(self, input_points, xi):
		"""Project from the input ("sensor") plane to the unit sphere
		- input_points : ndarray[2, N] : Coordinates in the metric plane associated to the image (input frame)
		- xi           : float         : Mei’s model center shifting parameter (ξ in the paper)
		<------------------ ndarray[3, N] : 3D coordinates of the points’ projection on the unit sphere"""
		projected_x, projected_y = input_points

		# Equations directly taken from Christopher Mei’s paper (h⁻¹(mᵤ) = ...)
		# There are no other distortion factors so we can go directly from the image plane to the sphere
		return np.asarray((
			projected_x * (xi + np.sqrt(1 + (1-xi**2)*(projected_x**2 + projected_y**2))) / (projected_x**2 + projected_y**2 + 1),
			projected_y * (xi + np.sqrt(1 + (1-xi**2)*(projected_x**2 + projected_y**2))) / (projected_x**2 + projected_y**2 + 1),
			(xi + np.sqrt(1 + (1-xi**2)*(projected_x**2 + projected_y**2))) / (projected_x**2 + projected_y**2 + 1) - xi,
		))

	def sphere_to_angles(self, sphere_points):
		"""Convert 3D cartesian coordinates on the surface of the unit sphere to spheric coordinates angles (colatitude, longitude)
		- sphere_points : ndarray[3, N] : 3D coordinates on the surface of the unit sphere
		<------------------- ndarray[2, N] : Spheric coordinates angles (colatitude θ, longitude φ)"""

		# We are on a unit sphere, so the radius is 1
		# Those are traditional physics-convention spheric coordinates (radius, colatitude, longitude)
		# Except at this stage we don’t know the actual radius of the original 3D point yet
		return np.asarray((
			np.arccos(sphere_points[2]),                     # Colatitude θ = arccos(z/ρ), ρ = 1
			np.arctan2(sphere_points[1], sphere_points[0]),  # Longitude  φ = atan2(y/x)
		))

	def angles_to_camera(self, angles_points, camera_to_target):
		"""Retrieve the original 3D point in the camera frame from their spheric coordinates angles relative to the camera unit sphere
		- angles_points    : ndarray[2, N] : Spheric coordinates angles (colatitude θ, longitude φ, the radius is assumed to be 1)
		- camera_to_target : ndarray[4, 4] : Transform matrix from the camera frame to the target frame where all points’ Z coordinate is null
		<---------------------- ndarray[4, N] : Homogeneous 3D coordinates of the points in the camera frame"""
		colatitude, longitude = angles_points

		# When we convert those points in the camera frame to the target frame (camera_to_target @ camera_points),
		# the resulting points have their Z coordinate equal to 0
		# So according to the matrix product, aX + bY + cZ + d = 0, with [a, b, c, d] the 3rd row of camera_to_target
		# (that, multiplied with a point vector, makes its target Z coordinate)
		# Those are our plane’s parameters in the camera frame
		a, b, c, d = camera_to_target[2]

		# Then we take the system that converts from spheric coordinates to cartesian and add the plane equation,
		# solving it for the radius ρ with the plane equation, then the 3D camera frame coordinates X, Y, Z
		# | X = ρ·sin(θ)·cos(φ)
		# | Y = ρ·sin(θ)·sin(φ)
		# | Z = ρ·cos(θ)
		# | aX + bY + cZ + d = 0
		camera_radius = -d / (a*np.sin(colatitude)*np.cos(longitude) + b*np.sin(colatitude)*np.sin(longitude) + c*np.cos(colatitude))
		camera_x = camera_radius * np.sin(colatitude) * np.cos(longitude)
		camera_y = camera_radius * np.sin(colatitude) * np.sin(longitude)
		camera_z = camera_radius * np.cos(colatitude)
		camera_points = np.asarray((camera_x, camera_y, camera_z, np.ones(angles_points.shape[1])))
		return camera_points

	def camera_to_target_(self, camera_points, camera_to_target):
		"""Convenience function that converts the 3D homogeneous points in the camera frame to the target frame. Only does `camera_to_target @ camera_points`
		- camera_points    : ndarray[4, N] : Homogeneous 3D coordinates of the points in the camera frame
		- camera_to_target : ndarray[4, 4] : Transform matrix from the camera frame to the target frame where all points’ Z coordinate is null
		<---------------------- ndarray[4, N] : Homogeneous 3D coordinates of the points in the target frame. All Z coordinates are 0, give or take floating-point shenanigans."""
		target_points = camera_to_target @ camera_points

	def target_to_output(self, target_points, x_range, y_range, output_size_y, flip_x=False, flip_y=False):
		"""Make a bird-eye view image from the points in the target frame.
		Make an orthogonal projection relative to the target Z axis (so parallel to the target plane)
		Warning : the resulting coordinates are not rounded to integers and may lie outside of the image
		- target_points : ndarray[4, N] : Homogeneous 3D coordinates of the points in the target frame
		- x_range       : [xmin, xmax]  : Metric range to display on the image’s x axis (for instance, [-25, 25] makes an image that shows all points within 25 meters on each side of the origin)
		- y_range       : [ymin, ymax]  : Metric range to display on the image’s y axis (for instance, [0, 50] makes an image that shows all points from the origin to 50 meters forward)
		- output_size_y : int           : Height of the output image in pixels. The width is calculated accordingly to scale points right
		- flip_x=False  : bool          : If `True`, reverse the X axis
		- flip_y=False  : bool          : If `True`, reverse the Y axis
		<------------------- ndarray[2, N] : Pixel coordinates of the given points for the output image described by the parameters
		<------------------- float         : Scale factor from pixels to metric (pixels × scale_factor = metric. If flip_x or flip_y are set, don’t forget to flip it back beforhand, scale_factor * (height-y) or scale_factor * (width-x)"""

		# Same scale on the X and Y axes -> calculate the width according to X and Y ranges
		output_size_x = output_size_y * (x_range[1] - x_range[0]) / (y_range[1] - y_range[0])

		# Scale and translate accordingly
		scale_factor = output_size_y / (y_range[1] - y_range[0])
		output_x = scale_factor * (target_points[0] - x_range[0])
		output_y = scale_factor * (target_points[1] - y_range[0])
		if flip_x:
			output_x = output_size_x - output_x
		if flip_y:
			output_y = output_size_y - output_y
		output_points = np.asarray((output_x, output_y))
		return output_points, 1/scale_factor
	
	def _postprocess_gray(self, output_image):
		"""In-place bilinear filtering for bird-eye view images"""
		for x in range(0, output_image.shape[1]):
			set_pixels = np.nonzero(output_image[:, x] > 0)[0]
			if set_pixels.shape[0] == 0:
				continue
			start = np.min(set_pixels)
			end = np.max(set_pixels)
			visual_range = np.arange(start, end, 1)
			
			output_image[visual_range, x] = np.interp(visual_range, set_pixels, output_image[set_pixels, x])

		for y in range(0, output_image.shape[0]):
			set_pixels = np.nonzero(output_image[y, :] > 0)[0]
			if set_pixels.shape[0] == 0:
				continue
			start = np.min(set_pixels)
			end = np.max(set_pixels)
			visual_range = np.arange(start, end, 1)

			output_image[y, visual_range] = np.interp(visual_range, set_pixels, output_image[y, set_pixels])


	def _postprocess(self, output_image):
		"""In-place bilinear filtering for bird-eye view images"""
		for x in range(0, output_image.shape[1]):
			set_pixels = np.nonzero((output_image[:, x, 0] + output_image[:, x, 1] + output_image[:, x, 2]) > 0)[0]
			if set_pixels.shape[0] == 0:
				continue
			start = np.min(set_pixels)
			end = np.max(set_pixels)
			visual_range = np.arange(start, end, 1)
			
			output_image[visual_range, x, 0] = np.interp(visual_range, set_pixels, output_image[set_pixels, x, 0])
			output_image[visual_range, x, 1] = np.interp(visual_range, set_pixels, output_image[set_pixels, x, 1])
			output_image[visual_range, x, 2] = np.interp(visual_range, set_pixels, output_image[set_pixels, x, 2])
		for y in range(0, output_image.shape[0]):
			set_pixels = np.nonzero((output_image[y, :, 0] + output_image[y, :, 1] + output_image[y, :, 2]) > 0)[0]
			if set_pixels.shape[0] == 0:
				continue
			start = np.min(set_pixels)
			end = np.max(set_pixels)
			visual_range = np.arange(start, end, 1)

			output_image[y, visual_range, 0] = np.interp(visual_range, set_pixels, output_image[y, set_pixels, 0])
			output_image[y, visual_range, 1] = np.interp(visual_range, set_pixels, output_image[y, set_pixels, 1])
			output_image[y, visual_range, 2] = np.interp(visual_range, set_pixels, output_image[y, set_pixels, 2])

	# equivalent to birdeye_to_target in cython version
	def image_to_target(self, image_points, camera_to_image, camera_to_target, xi):
		"""Directly calculate the 3D coordinates in the target frame of the given points in input image pixel coordinates
		- image_points     : ndarray[2, N] : Pixel coordinates in the image (vectors as columns)
		- camera_to_image  : ndarray[3, 3] : Projection matrix (K) to pixel coordinates
		- camera_to_target : ndarray[4, 4] : Transform matrix from the camera frame to the target frame where all points’ Z coordinate is null
		- xi               : float         : Mei’s model center shifting parameter (ξ in his paper)
		<---------------------- ndarray[4, N] : Homogeneous 3D coordinates of the points in the target frame. All Z coordinates are 0, give or take floating-point shenanigans."""
		input_points = self.image_to_input(image_points, camera_to_image)
		sphere_points = self.input_to_sphere(input_points, xi)
		angles_points = self.sphere_to_angles(sphere_points)
		camera_points = self.angles_to_camera(angles_points, camera_to_target)
		target_points = camera_to_target @ camera_points
		return target_points

	def init_projection(self):
		"""Directly creates a bird-eye view from an image
		- image            : ndarray[y, x] : Original image
		- camera_to_image  : ndarray[3, 3] : Projection matrix (K) to pixel coordinates
		- camera_to_target : ndarray[4, 4] : Transform matrix from the camera frame to the target frame where all points’ Z coordinate is null
		- xi               : float         : Mei’s model center shifting parameter (ξ in his paper)
		- x_range          : [xmin, xmax]  : Metric range to display on the image’s x axis (for instance, [-25, 25] makes an image that shows all points within 25 meters on each side of the origin)
		- y_range          : [ymin, ymax]  : Metric range to display on the image’s y axis (for instance, [0, 50] makes an image that shows all points from the origin to 50 meters forward)
		- output_size_y    : int           : Height of the output image in pixels. The width is calculated accordingly to scale points right
		- flip_x=False     : bool          : If `True`, reverse the X axis
		- flip_y=False     : bool          : If `True`, reverse the Y axis
		<---------------------- ndarray[y, x] : Resulting bird-eye view image (with bilinear interpolation)
		<---------------------- float         : Scale factor from pixels to metric (pixels × scale_factor = metric. If flip_x or flip_y are set, don’t forget to flip it back beforhand, scale_factor * (height-y) or scale_factor * (width-x)"""
		self.image_points = self.get_image_points(self.input_image_shape)
		target_points = self.image_to_target(self.image_points, self.camera_to_image, self.camera_to_target, self.xi)
		output_points, self.scale_factor = self.target_to_output(target_points, self.x_range, self.y_range, self.output_size_y, self.flip_x, self.flip_y)
		self.output_pixels = output_points.astype(int)
		output_size_x = int(self.output_size_y * (self.x_range[1] - self.x_range[0]) / (self.y_range[1] - self.y_range[0]))

		# Filter out points outside of the resulting image
		self.output_filter = (0 <= self.output_pixels[0]) & (self.output_pixels[0] < output_size_x) & (0 <= self.output_pixels[1]) & (self.output_pixels[1] < self.output_size_y)

		# To make it work for all image formats
		self.output_shape = (self.output_size_y, output_size_x) + self.input_image_shape[2:]


	def to_birdeye(self, image):
		output_image = np.zeros(self.output_shape, dtype=image.dtype)
		output_image[self.output_pixels[1, self.output_filter], self.output_pixels[0, self.output_filter]] = image[self.image_points[1, self.output_filter], self.image_points[0, self.output_filter]]
		
		if(len(self.output_shape) == 2):
			self._postprocess_gray(output_image)
		else:
			self._postprocess(output_image)
		return output_image, self.scale_factor


if __name__ == "__main__":
	
	img = cv.imread('image.png')

	img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
	
	# plt.imshow(img, cmap='gray')
	# plt.show()

	K = np.array([  [1124.66943359375, 0.0, 505.781982421875],
	[0.0, 1124.6165771484375, 387.8110046386719],
	[0.0, 0.0, 1.0]])

	T_cam_to_work_plane = np.array([[ 0.99991969 , 0.01149682 ,-0.00533206 , 0.05419943],
	[ 0.00510958 , 0.01929436 , 0.99980079 , 1.96159697],
	[ 0.0115974  ,-0.99974774  ,0.01923406  ,1.55057154],
	[ 0.          ,0.          ,0.          ,1.        ]])

	xi = 0.8803200125694275

	print(img.shape)

	bv = Fish2Bird(img.shape, K, T_cam_to_work_plane, xi, [-25, 25], [0, 50], 500)

	# Convert the image to a bird's eye view
	output_image, scale_factor = bv.to_birdeye(img)

	print(bv.get_output_shape())

	# Show the result
	output_image = cv.cvtColor(output_image, cv.COLOR_BGR2RGB)
	plt.imshow(output_image)
	plt.show()