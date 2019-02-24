import math
import rendering

"""
Implements a point
"""
class Point:

	def __init__(self, x, y):
		self.x = x
		self.y = y

	def move(self, dx, dy):
		return Point(self.x + dx, self.y + dy)

	def move_seg_by_angle(self, angle, dis):
		angle = to_radian(angle)
		to = self.move(dis * math.cos(angle), dis * math.sin(angle))
		return LineSegment(self, to)

	def diff(self, fr):
		return Vector(self.x - fr.x, self.y - fr.y)

	def angle_to(self, to):
		if self.x == to.x:
			if self.y > to.y:
				return 270
			return 90
		return to_degree(to.diff(self).angle_radian())

	def float_equals(self, other):
		return float_equals(self.x, other.x) and float_equals(self.y, other.y)

	def dis(self, to):
		return math.sqrt((self.x - to.x) ** 2 + (self.y - to.y) ** 2)

	def midpoint(self, other):
		return Point((self.x + other.x) / 2, (self.y + other.y) / 2)

	def four_split(self, other):
		mid = self.midpoint(other)
		return [self.midpoint(mid) + mid + mid.midpoint(other)]

	def to_list(self):
		return [self.x, self.y]

	def __repr__(self):
		return "{0}, {1}".format(self.x, self.y)


"""
Contains vector functions for convenience
"""
class Vector(Point):

	def __init__(self, x, y):
		super().__init__(x, y)
		self.length = self.dis(Point(0, 0))

	def dot(self, other):
		return self.x * other.x + self.y * other.y

	def scale(self, factor):
		return Vector(self.x * factor, self.y * factor)

	def normalize(self):
		if self.length == 0:
			return Vector(0, 0)
		return self.scale(1 / self.length)

	def project(self, other):
		return other.normalize().scale(self.dot(other) / other.length)

	def angle_radian(self):
		deg = math.atan(self.y / self.x)
		if self.x < 0:
			deg += math.pi
		if deg < 0:
			deg += 2 * math.pi
		return deg


class LineSegment:

	def __init__(self, point_from, point_to):
		self.point_from = point_from
		self.point_to = point_to
		self.move_vec = point_to.diff(point_from)
		self.midpoint = point_from.midpoint(point_to)
		if point_from.x == point_to.x:
			self.slope = float('inf')
		else:
			self.slope = (point_to.y - point_from.y) / (point_to.x - point_from.x)

	def y_at(self, x):
		if self.slope == float('inf'):
			return float('inf')
		return self.point_from.y + (x - self.point_from.x) * self.slope

	def length(self):
		return self.point_from.dis(self.point_to)

class Team:

	def __init__(self, name, pygame_rendering):
		self.name = name
		self.robots = []
		self.enemy = None

		if self.name == "BLUE":
			if pygame_rendering:
				self.dark_color = PYGAME_COLOR_DARKBLUE
				self.color = PYGAME_COLOR_BLUE
			else:
				self.dark_color = COLOR_DARKBLUE
				self.color = COLOR_BLUE
		else:
			if pygame_rendering:
				self.dark_color = PYGAME_COLOR_DARKRED
				self.color = PYGAME_COLOR_RED
			else:
				self.dark_color = COLOR_DARKRED
				self.color = COLOR_RED

	def add_robot(self, robot):
		self.robots.append(robot)

	def add_defense_buff(self, time):
		for r in self.robots:
			r.add_defense_buff(time)

	def total_health(self):
		return sum([r.health for r in self.robots])

	def set_health_bar(self, rec, viewer=None):
		self.bar = rec
		if len(self.robots) == 1:
			self.robots[0].health_bar = rec
			if viewer:
				viewer.add_geom(rendering.PolyLine([p.to_list() for p in rec.vertices], True))
		else:
			left_middle = rec.vertices[0].midpoint(rec.vertices[3])
			self.robots[0].health_bar = type(rec)(rec.vertices[0], \
			    rec.width, rec.height/2)
			self.robots[1].health_bar = type(rec)(left_middle, \
			    rec.width, rec.height/2)
			if viewer:
				viewer.add_geom(rendering.PolyLine([p.to_list() for p in self.robots[0].health_bar.vertices], True))
				viewer.add_geom(rendering.PolyLine([p.to_list() for p in self.robots[1].health_bar.vertices], True))

	def generate_state(self):
		state = []
		for r in self.robots:
			state += r.generate_state()
		if len(self.robots) == 1:
			length = len(state)
			state += [0] * length
		return state

def to_radian(deg):
	return deg / 180 * math.pi

def to_degree(rad):
	return rad * 180 / math.pi

def float_equals(a, b, error_threshold=0.01):
	return abs(a - b) < error_threshold

def sign(x):
	if x > 0:
		return 1
	if x < 0:
		return -1
	return 0


COLOR_BLUE = (0, 0, 1)
COLOR_DARKBLUE = (0, 0, 0.5)
COLOR_RED = (1, 0, 0)
COLOR_DARKRED = (0.5, 0, 0)
COLOR_GREEN = (0, 1, 0)
COLOR_BLACK = (0, 0, 0)
COLOR_WHITE = (1, 1, 1)
COLOR_YELLOW = (1, 1, 0)

PYGAME_COLOR_WHITE = (255, 255, 255)
PYGAME_COLOR_RED = (255, 0, 0)
PYGAME_COLOR_BLUE = (0, 0, 255)
PYGAME_COLOR_DARKRED = (127, 0, 0)
PYGAME_COLOR_DARKBLUE = (0, 0, 127)
PYGAME_COLOR_YELLOW = (255, 255, 0)