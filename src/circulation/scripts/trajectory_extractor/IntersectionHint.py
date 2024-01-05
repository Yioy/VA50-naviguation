import numpy as np

import trajectory_extractor.circulation_enums as enums

class IntersectionHint (object):
	"""Hold the position and informations about anything that may indicate an intersection"""

	def __init__(self, category, type, position, timestamp, confidence):
		self.category = category
		self.type = type
		self.positions = [position]
		self.position_timestamps = [timestamp]
		self.confidences = [confidence]
	
	def merge(self, hint):
		self.positions.extend(hint.positions)
		self.position_timestamps.extend(hint.position_timestamp)
		self.confidences.extend(hint.confidences)
	
	def confidence(self):
		return 1 - np.sqrt(np.sum((1 - np.asarray(self.confidences))**2)) / len(self.confidences)

	def direction_hint(self):
		if self.category != "trafficsign":
			return enums.Direction.FORWARD | enums.Direction.LEFT | enums.Direction.RIGHT

		if self.type in ("right-only", "keep-right"):
			return enums.Direction.RIGHT
		elif self.type in ("left-only", "keep-left"):
			return enums.Direction.LEFT
		elif self.type == "ahead-only":
			return enums.Direction.FORWARD
		elif self.type == "straight-left-only":
			return enums.Direction.FORWARD | enums.Direction.LEFT
		elif self.type == "straight-right-only":
			return enums.Direction.FORWARD | enums.Direction.RIGHT
		else:
			return enums.Direction.FORWARD | enums.Direction.LEFT | enums.Direction.RIGHT

	def __hash__(self):
		return id(self)

	def __str__(self):
		return f"{self.category} > {self.type} : {self.positions}, {self.confidences}"
	def __repr__(self):
		return f"{self.category} > {self.type} : {self.positions}, {self.confidences}"