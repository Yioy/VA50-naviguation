import rospy
import numpy as np
from threading import Lock

from transformtrack.srv import TransformBatch, TransformBatchRequest, DropVelocity, DropVelocityRequest

class TransformServiceHandler(object):
	"""Handles requests to the transform service."""

	def __init__(self, transform_service_name, drop_service_name):

		self.transform_service_name = transform_service_name
		self.drop_service_name = drop_service_name

		# Initialize the service connections
		rospy.loginfo("Waiting for the TransformBatch service...")
		self.transform_service = None
		self.drop_service = None
		rospy.wait_for_service(self.transform_service_name)
		rospy.wait_for_service(self.drop_service_name)
		self.transform_service = rospy.ServiceProxy(self.transform_service_name, TransformBatch, persistent=True)
		self.drop_service = rospy.ServiceProxy(self.drop_service_name, DropVelocity, persistent=True)
		self.transform_service_lock = Lock()



	def get_map_transforms(self, start_times, end_time):
		"""Get a batch of transforms of the vehicle frame from the given `start_times` to `end_time`
		   - start_times : list<rospy.Time> : Timestamps to get the transforms from
		   - end_time    : rospy.Time       : Target timestamp, to get the transforms to
		<----------------- ndarray[N, 4, 4] : 3D homogeneous transform matrices to transform points in the vehicle frame
											  at `start_times[i]` to the vehicle frame at `end_time`
		<----------------- list<rospy.Time> : Unbiased start times. This is an artifact from the time when the simulator gave incoherent timestamps,
											  same as `start_times` on the latest versions
		<----------------- rospy.Time       : Unbiased end time, same as end_time on the latest versions of the simulator
		"""
		# Build the request to the transform service, see srv/TransformBatch.srv and msg/TimeBatch.msg for info
		request = TransformBatchRequest()
		request.start_times = start_times
		request.end_time = end_time

		# Now send the request and get the response
		# We use persistent connections to improve efficiency, and ROS advises to implement some reconnection logic
		# in case the network gets in the way, so in case of disconnection, retry 10 times to reconnect then fail
		tries = 0
		while True:
			try:
				# Apparently, when a call to the service is pending, the node is free to service other callbacks,
				# including callback_trafficsign that also call this service
				# So with the traffic signs subscriber active, it’s only a matter of time until both get to their transform service call concurrently
				# For some reason, ROS allows it, and for some reason it deadlocks ROS as a whole
				# So let’s throw in a lock to prevent ROS from killing itself
				with self.transform_service_lock:
					response = self.transform_service(request)
				break
			except rospy.ServiceException as exc:
				if tries > 10:
					rospy.logerr(f"Connection to service {self.transform_service_name} failed {tries} times, skipping")
					rospy.logerr(f"Failed with error : {exc}")
					raise RuntimeError("Unable to connect to the transform service")
				rospy.logerr(f"Connection to service {self.transform_service_name} lost, reconnecting...")
				self.transform_service.close()
				self.transform_service = rospy.ServiceProxy(self.transform_service_name, TransformBatch, persistent=True)
				tries += 1
		
		# The call was successful, get the transforms in the right format and return
		# The transpose is because the individual matrices are transmitted in column-major order
		transforms = np.asarray(response.transforms.data).reshape(response.transforms.layout.dim[0].size, response.transforms.layout.dim[1].size, response.transforms.layout.dim[2].size).transpose(0, 2, 1)
		distances = np.asarray(response.distances)
		return transforms, distances
	
	def drop_velocity(self, end_time):
		"""Call the DropVelocity service, such that the TransformBatch service discards its old velocity data
		   and doesn’t unnecessarily clutter its memory and performance
		   - end_time : rospy.Time : Discard all velocity data prior to this timestamp
		"""
		request = DropVelocityRequest(end_time=end_time, unbias=True)
		
		# Same reconnection logic as .get_map_transforms
		tries = 0
		while True:
			try:
				response = self.drop_service(request)
				break
			except rospy.ServiceException as exc:
				if tries > 10:
					rospy.logerr(f"Connection to service {self.drop_service_name} failed {tries} times, skipping")
					rospy.logerr(f"Failed with error : {exc}")
					raise RuntimeError("Unable to connect to the velocity drop service")
				rospy.logerr(f"Connection to service {self.drop_service_name} lost, reconnecting...")
				self.drop_service.close()
				self.drop_service = rospy.ServiceProxy(self.drop_service_name, DropVelocity, persistent=True)
				tries += 1
	
