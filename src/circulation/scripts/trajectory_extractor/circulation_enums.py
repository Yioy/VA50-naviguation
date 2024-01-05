import enum

INTERSECTION_SIGNS = ( 
    'yield', 
    'stop', 
    'right-only', 
    'left-only', 
    'ahead-only', 
    'straight-right-only', 
    'straight-left-only', 
    'keep-right', 
    'keep-left',
)

TURN_SIGNS = (
	'right-only', 
    'left-only', 
    'ahead-only', 
    'straight-right-only', 
    'straight-left-only', 
    'keep-right', 
    'keep-left',
)


class NavigationMode (enum.Enum):
	CRUISE = 0
	INTERSECTION_FORWARD = 110
	INTERSECTION_LEFT = 111
	INTERSECTION_RIGHT = 112
	TURN_LEFT = 113
	TURN_RIGHT = 114
	PANIC_CORE_BREACH = 500
	PANIC_UNSUPPORTED = 501
	PANIC_EXCEPTION = 502
	PANIC_NO_DIRECTION = 510

	def is_intersection(self):
		return self == NavigationMode.INTERSECTION_LEFT or self == NavigationMode.INTERSECTION_RIGHT or self == NavigationMode.INTERSECTION_FORWARD

class Direction:
	DEAD_END = 0b0000
	FORWARD = 0b0001
	LEFT = 0b0010
	RIGHT = 0b0100
	DOUBLE_LANE = 0b1000
	FORCE_INTERSECTION = 0b10000