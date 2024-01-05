import numpy as np
import fuzzylines


class TrajectoryExtractor (object):
	"""Main class for the ROS node that manages the trajectory"""

	#                        ╔══════════════════════╗                       #
	# ═══════════════════════╣    INITIALISATION    ╠══════════════════════ #
	#                        ╚══════════════════════╝                       #

	def __init__(self, parameters):
		"""Initialize the node and everything that it needs
		   - parameters   : dict<str: …>        : Node parameters, from the parameter file
		"""
		self.parameters = parameters
		
		# Initialize the fuzzy systems
		self.init_fuzzysystems()
	
	
	def init_fuzzysystems(self):
		"""Initialize the fuzzy systems used by the lane detection"""
		line_variables = ("forward-distance", "line-distance", "line-lengths", "parallel-distances", "parallel-angles")
		line_centers = np.asarray([self.parameters["fuzzy-lines"]["centers"][variable] for variable in line_variables])
		line_malus = np.asarray([self.parameters["fuzzy-lines"]["malus"][variable] for variable in line_variables], dtype=int)
		line_output_centers = np.asarray(self.parameters["fuzzy-lines"]["centers"]["output"])
		self.lane_system = fuzzylines.FuzzySystem(line_centers, line_malus, line_output_centers, self.parameters["fuzzy-lines"]["base-score"])
		

	

