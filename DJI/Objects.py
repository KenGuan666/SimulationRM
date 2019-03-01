"""
Describes all objects present in the challenge field
"""

import numpy as np
import math
import heapq
import cv2
import rendering
import keyboard
import pygame

from utils import *
from strategy import *

"""
Every object is an implementation of the abstract class Character
"""
class Character:

	color = COLOR_BLACK

	def act(self, env):
		pass

	def render(self):
		pass

	def pygame_render(self, screen):
		pass

	def reset(self):
		pass


"""
All rectangular objects are described by class Rectangle

defined by the rectangle's center, width(x-span), height(y-span) and angle in degrees
"""
class Rectangle(Character):

	def __init__(self, bottom_left, width, height, angle=0):
		self.bottom_left = bottom_left
		self.width = width
		self.height = height
		self.set_angle(angle)
		self.angle_radian = angle / 180 * math.pi
		self.vertices = self.get_vertices()
		self.sides = self.get_sides()
		self.center = self.get_center()

	def by_center(center, width, height, angle=0):
		helper_rec = Rectangle(center, width / 2, height / 2, angle + 180)
		return Rectangle(helper_rec.vertices[2], width, height, angle)

	def get_vertices(self):
		width_dx = math.cos(self.angle_radian) * self.width
		width_dy = math.sin(self.angle_radian) * self.width

		height_dx = -math.sin(self.angle_radian) * self.height
		height_dy = math.cos(self.angle_radian) * self.height

		bottom_right = self.bottom_left.move(width_dx, width_dy)
		top_left = self.bottom_left.move(height_dx, height_dy)
		top_right = bottom_right.move(height_dx, height_dy)

		return [self.bottom_left, bottom_right, top_right, top_left]

	def get_sides(self):
		return [LineSegment(self.vertices[i], self.vertices[(i + 1) % 4]) for i in range(4)]

	def get_center(self):
		return self.bottom_left.midpoint(self.vertices[2])

	def set_angle(self, deg):
		self.angle = deg % 360
		self.angle_radian = deg / 180 * math.pi

	def project_points(self, points):
		width_vec = self.vertices[1].diff(self.bottom_left)
		height_vec = self.vertices[3].diff(self.bottom_left)

		ans = []
		for p in points:
			goal_vec = p.diff(self.bottom_left)
			width_proj = goal_vec.project(width_vec)
			height_proj = goal_vec.project(height_vec)
			x = sign(width_proj.dot(width_vec)) * width_proj.length
			y = sign(height_proj.dot(height_vec)) * height_proj.length
			ans.append(Point(x, y))

		return ans

	"""
	Check if a point is contained by self. Uses some messy linalg. Please Suggest
	better implementation if possible.
	"""
	def contains(self, point, error_threshold=0.01):
		point = self.project_points([point])[0]
		return point.x - self.width < error_threshold and \
	       point.y - self.height < error_threshold and \
	       point.x >= 0 and point.y >= 0

	def contains_any(self, points):
		for p in points:
			if self.contains(p):
				return True
		return False

	def contains_all(self, points):
		for p in points:
			if not self.contains(p):
				return False
		return True

	"""
	Checks if two rectangles intersect
	IT DOESN'T CONSIDER SOME CASES which I don't think are necessary for our app
	"""
	def intersects(self, other):
		# if self.contains_any(other.vertices) or other.contains_any(self.vertices):
		# 	return True
		for side in other.sides:
			if self.blocks(side):
				return True
		return False

	def blocks(self, seg):
		points = self.project_points([seg.point_from, seg.point_to])
		helper_upright = UprightRectangle(Point(0, 0), self.width, self.height)
		return helper_upright.blocks(LineSegment(points[0], points[1]))

	def angle_to(self, point):
		return self.center.angle_to(point)

	"Renders the rectangle depending on type"
	def render(self, color=None):
		rec = rendering.FilledPolygon([p.to_list() for p in self.vertices])
		if not color:
			color = self.color
		rec.set_color(color[0], color[1], color[2])
		return [rec]

	def pygame_render(self, screen, color=None):
		if not color:
			color = self.color
		pygame.draw.polygon(screen, color, [p.to_list() for p in self.vertices])


"""
Type of rectangle that never rotates.
Has implementation of certain methods that are more efficient
"""
class UprightRectangle(Rectangle):

	def get_vertices(self):
		bottom_left = self.bottom_left
		bottom_right = bottom_left.move(self.width, 0)
		top_right = bottom_right.move(0, self.height)
		top_left = bottom_left.move(0, self.height)
		self.left = bottom_left.x
		self.right = top_right.x
		self.top = top_right.y
		self.bottom = bottom_left.y
		return [bottom_left, bottom_right, top_right, top_left]

	def contains(self, point):
		x, y = point.x, point.y
		return self.left <= x and self.bottom <= y and self.right >= x and self.top >= y

	def blocks(self, seg):
		point_from, point_to = seg.point_from, seg.point_to
		if self.contains(point_from) or self.contains(point_to):
			return True
		if (point_from.x < self.left and point_to.x < self.left) or \
		   (point_from.x > self.right and point_to.x > self.right) or \
		   (point_from.y < self.bottom and point_to.y < self.bottom) or \
		   (point_from.y > self.top and point_to.y > self.top):
		   return False
		if point_from.x == point_to.x and \
			((point_from.y > self.top and point_to.y < self.top) or \
			(point_from.y < self.top and point_to.y > self.top)):
			return True
		
		left_y, right_y = seg.y_at(self.left), seg.y_at(self.right)

		return not (left_y > self.top and right_y > self.top) and \
		       not (left_y < self.bottom and right_y < self.bottom)

	def pygame_render(self, screen, color=None):
		if not color:
			color = self.color
		pygame.draw.rect(screen, color, [self.left, self.bottom, self.width, self.height])

"""
Impermissible and inpenetrable obstacles are described by class Obstacle
"""
class Obstacle(UprightRectangle):

	type = 'OBSTACLE'

	def permissible(self, team):
		return False

	def penetrable(self):
		return False


"""
Permissble and penetrable areas in the field are described by class Zone
"""
class Zone(UprightRectangle):

	type = 'ZONE'

	sidelength = 100

	def __init__(self, bottom_left, team):
		super().__init__(bottom_left, 100, 100)
		self.team = team
		self.color = team.dark_color
		self.render_helper = UprightRectangle(self.bottom_left.move(8, 8), \
		    self.width - 16, self.height - 16)

	def penetrable(self):
		return True

	def permissble(self, team):
		return True

	def render(self):
		return super().render() + self.render_helper.render(COLOR_WHITE)

	def pygame_render(self, screen):
		super().pygame_render(screen)
		self.render_helper.pygame_render(screen, PYGAME_COLOR_WHITE)

	def generate_state(self):
		return []


"""
Loading zones that provide 17mm bullets
"""
class LoadingZone(Zone):

	def __init__(self, bottom_left, team, env):
		super().__init__(bottom_left, team)
		team.loading_zone = self
		self.loading_point = self.center
		self.env = env
		self.loaded = 0
		self.to_load = 0
		self.rendering_center = [int(x) for x in self.center.to_list()]
			

	fills = 2
	tolerance_radius = 3
	load_speed = 20 # per sec

	"""
	Enemy loading zone is modeled as impermissible
	"""
	def permissble(self, team):
		return self.team == team

	"""
	Checks if the robot is aligned with the bullet supply machinary
	"""
	def aligned(self, robot):
		return float_equals(self.loading_point.x, robot.center.x, self.tolerance_radius) and \
		    float_equals(self.loading_point.y, robot.center.y, self.tolerance_radius)

	def act(self):
		if self.loading():
			load_per_step = self.load_speed * self.env.tau
			self.to_load -= load_per_step
			for r in self.team.robots:
				if self.aligned(r):
					self.loaded, before = self.loaded + load_per_step, self.loaded
					diff = math.floor(self.loaded) - math.floor(before)
					if diff > 0:
						r.load(diff)
		else:
			self.to_load = 0
			self.loaded = 0
		if any([self.aligned(r) for r in self.team.robots]):
			self.color = self.team.color
		else:
			self.color = self.team.dark_color

	def load(self):
		if self.fills <= 0:
			print("Team " + self.team.name + " has no more reloads available.")
			return
		print("Reload command successful.")
		self.to_load += 100
		self.fills -= 1

	def loading(self):
		return self.to_load > 0

	def reset(self):
		self.fills = 2

	def render(self):
		circle = rendering.Circle(self.center, 12)
		circle.set_color(self.color[0], self.color[1], self.color[2])
		if self.loading() > 0:
			self.env.viewer.add_onetime_text("to_load: {0}, loaded: {1}".format(int(self.to_load), int(self.loaded)), \
			    10, self.center.x, self.center.y)
		return super().render() + [circle]
	
	def pygame_render(self, screen):
		super().pygame_render(screen)
		pygame.draw.circle(screen, self.color, self.rendering_center, 12)
		if self.loading() > 0:
			self.text = pygame.transform.flip(self.env.font.render("to_load: {0}, loaded: {1}".format(int(self.to_load), \
				int(self.loaded)), False, COLOR_BLACK), False, True)
		else:
			self.text = None

	def generate_state(self):
		return [self.fills, self.to_load, self.loaded]


"""
Buff zone that boosts defense for one team
"""
class DefenseBuffZone(Zone):

	# color =

	active = True

	def __init__(self, bottom_left, team, env):
		super().__init__(bottom_left, team)
		self.d_helper = Rectangle(self.center.move(0, -15), \
		    15 * 1.414, 15 * 1.414, 45)
		team.defense_buff_zone = self
		self.env = env
		self.touch_record = [0, 0, 0, 0]

	def act(self):
		touched = False
		for r in self.env.characters['robots']:
			if self.contains_all(r.vertices):
				touched = True
				self.touch_record[r.id] += self.env.tau
				if self.active and self.touch_record[r.id] >= 5:
					self.activate()
			else:
				self.touch_record[r.id] = 0
		if touched:
			self.color = self.team.color
		else:
			self.color = self.team.dark_color

	def activate(self):
		print(self.team.name + " team has activated defense buff!")
		self.team.add_defense_buff(30)
		self.active = False

	def reset(self):
		self.active = True

	def render(self):
		return super().render() + self.d_helper.render(self.color)
	
	def pygame_render(self, screen):
		super().pygame_render(screen)
		self.d_helper.pygame_render(screen, self.color)

	def generate_state(self):
		active_flag = 0
		if self.active:
			active_flag = 1
		return [active_flag] + self.touch_record


class StartingZone(Zone):
	pass


"""
Describes a bullet. Currently modeled as having constant speed and no volume
"""
class Bullet(Character):

	damage = 50

	def __init__(self, point, dir, env, master):
		self.delay = 0  # Models the delay from firing decision to bullet actually flying
		self.speed = 25
		self.point = point
		self.dir = dir / 180 * math.pi
		self.env = env
		self.active = True
		self.master = master
		self.range = master.range

	def init_from_state(state, env):
		self = Bullet(Point(state[0], state[1]), 0, env, env.characters['robots'][state[3]])
		self.dir = state[2]
		return self

	def act(self):
		if not self.active:
			return
		if self.delay > 0:
			self.delay -= 1
			return
		move_point = self.point.move(math.cos(self.dir) * self.speed, math.sin(self.dir) * self.speed)
		move_seg = LineSegment(self.point, move_point)

		blocker = self.env.is_blocked(move_seg, [self.master])
		if blocker:
			self.destruct()
			if blocker.type == 'ARMOR':
				blocker.master.reduce_health(self.damage)
		elif self.env.is_legal(move_point):
			self.point = move_point
			self.range -= self.speed
			if self.range <= 0:
				self.destruct()
		else:
			self.destruct()

	"""
	SUBJECT TO CHANGE!
	"""
	def destruct(self):
		self.env.characters['bullets'] = list(filter(lambda b: not b == self, self.env.characters['bullets']))
		self.active = False

	def render(self):
		return [rendering.Circle(self.point, 2)]

	def pygame_render(self, screen):
		pygame.draw.circle(screen, COLOR_BLACK, [int(x) for x in self.point.to_list()], 2)

	def generate_state(self):
		return [self.point.x, self.point.y, self.dir, self.master.id]


class Armor(Rectangle):

	type = 'ARMOR'

	def __init__(self, rec, robot):
		super().__init__(rec.bottom_left, rec.width, rec.height, rec.angle)
		self.master = robot

	def blocks(self, seg):
		return super().blocks(seg)


"""
The robot object -

To modify strategy, extend the class and override the `get_strategy` method
"""
class Robot(Rectangle):

	type = 'ROBOT'
	strategy_id = 0

	width = 55.0
	height = 42.0
	armor_size = 13.1
	health = 2000
	gun_width = height / 4
	gun_length = width
	range = 300 # More on this later
	bullet_capacity = 150

	max_speed = 311
	max_rotation_speed = 330
	max_gun_angle = 90
	max_gun_rotation_speed = 3
	max_cooldown = 0.2

	def __init__(self, env, team, bottom_left, angle=0):
		self.gun_angle = 0
		self.max_speed *= env.tau
		self.max_rotation_speed *= env.tau
		self.env = env
		self.team = team
		self.color = team.color
		team.add_robot(self)
		self.defense_buff_timer = 0
		super().__init__(bottom_left, self.width, self.height, angle)
		# self.heat = 0
		self.shooting = False
		self.cooldown = 0
		self.bullet = 0
		self.preset_action = None
		self.preset_timer = 0

	def init_from_state(state, env, team):
		robot_type = strats[state[9]]
		self = robot_type(env, team, Point(state[0], state[1]), state[2])
		self.gun_angle = state[3]
		self.bullet = state[4]
		self.cooldown = state[5] * env.tau
		self.defense_buff_timer = state[6] * env.tau
		if state[7]:
			self.shooting = True
		self.health = state[8]

		return self

	def render(self):
		if self.alive():
			self.health_display = UprightRectangle(self.health_bar.bottom_left.move(0, 1), \
			    self.health_bar.width - 1, self.health / Robot.health * (self.health_bar.height - 1))
			if self.has_defense_buff():
				self.health_display.color = COLOR_YELLOW
			else:
				self.health_display.color = self.color
			return super().render() + self.health_display.render() + \
			    self.get_gun().render(self.color) + [armor.render()[0] for armor in self.get_armor()]
		return super().render()

	def pygame_render(self, screen):
		if self.alive():
			self.health_display = UprightRectangle(self.health_bar.bottom_left.move(0, 1), \
			    self.health_bar.width - 1, self.health / Robot.health * (self.health_bar.height - 1))
			if self.has_defense_buff():
				self.health_display.color = PYGAME_COLOR_YELLOW
			else:
				self.health_display.color = self.color
			super().pygame_render(screen)
			self.health_display.pygame_render(screen)
			self.get_gun().pygame_render(screen, self.color)
			for armor in self.get_armor():
				armor.pygame_render(screen)
		super().pygame_render(screen)

	def alive(self):
		return self.health > 0

	def load(self, num):
		self.bullet = min(self.bullet + num, self.bullet_capacity)

	def has_defense_buff(self):
		return self.defense_buff_timer > 0

	def add_defense_buff(self, time):
		self.defense_buff_timer = time

	def get_enemy(self):
		enemy = self.team.enemy
		if enemy.robots[0].health == 0:
			return enemy.robots[1]
		return enemy.robots[0]

	"""
	Determine a strategy based on information in self.env
	"""
	def get_strategy(self):
		pass

	"""
	The function evoked by Environment each turn
	First decide on an strategy, then execute it
	"""
	def act(self):
		if self.alive():
			self.defense_buff_timer = max(0, self.defense_buff_timer - self.env.tau)
			self.cooldown = max(0, self.cooldown - self.env.tau)
			if self.preset_timer > 0:
				preset_timer -= self.env.tau
				for action_part in preset_action:
					action_part.resolve(self)
				return
			strategy = self.get_strategy()
			if strategy:
				action = strategy.decide_with_default(self)
				if action:
					for action_part in action:
						action_part.resolve(self)

	def set_position(self, rec):
		Rectangle.__init__(self, rec.bottom_left, rec.width, rec.height, rec.angle)

	def get_gun(self):
		bottom_left = self.vertices[1].midpoint(self.center).midpoint(self.center)
		return Rectangle(bottom_left, self.gun_length, self.gun_width, self.angle + self.gun_angle)

	def fire_line(self):
		return self.get_gun().center.move_seg_by_angle(self.angle + self.gun_angle, self.range)

	def aimed_at_enemy(self):
		line_block = self.env.is_blocked(self.fire_line(), [self])
		return line_block and (line_block.type == "ROBOT" and not (self.team is line_block.team) or \
		    line_block.type == "ARMOR" and not (self.team is line_block.master.team))

	def get_armor(self):
		return [Armor(Rectangle.by_center(self.vertices[i].midpoint(self.vertices[(i + 1) % 4]), \
		    self.armor_size, 3, self.angle + i % 2 * 90), self) for i in range(4)]

	def reduce_health(self, amount):
		if self.alive():
			if self.has_defense_buff():
				amount /= 2
			self.health = max(0, self.health - amount)

	def generate_state(self):
		shooting_flag = 0
		if self.shooting:
			shooting_flag = 1
		return [self.bottom_left.x, self.bottom_left.y, self.angle, self.gun_angle, self.bullet, \
		        int(self.cooldown / self.env.tau), int(self.defense_buff_timer / self.env.tau), \
				shooting_flag, self.health, self.strategy_id]


class DummyRobot(Robot):

	def get_strategy(self):
		return DoNothing()


class CrazyRobot(Robot):

	strategy_id = 1

	def get_strategy(self):
		return SpinAndFire()


class AttackRobot(Robot):

	strategy_id = 2

	def get_strategy(self):
		target = self.team.enemy.robots[0]
		return Attack()


class AttackWithRadiusRobot(Robot):
	pass


class KeyboardRobot(Robot):

	strategy_id = 2

	def __init__(self, controls, env, team, bottom_left, angle=0):
		super().__init__(env, team, bottom_left, angle)
		self.controls = controls
		self.pygame_rendering = env.pygame_rendering
		if env.rendering:
			self.strat = KeyboardPyglet(self.controls)
		elif env.pygame_rendering:
			self.strat = KeyboardPygame(self.controls)
			env.keyboard_robot = self
			env.listening = list(controls)
			self.actions = []

	def handle_key(self, key):
		if key in self.controls:
			if key in self.actions:
				self.actions.remove(key)
			else:
				self.actions.append(key)

	def get_strategy(self):
		if self.pygame_rendering:
			self.strat.set_instructions(self.actions)
		return self.strat


class TrainingRobot(Robot):

	def set_strategy(self, strat):
		self.strat = strat

	def get_strategy(self):
		return self.strat


class JoystickRobot(Robot):

	type = 'JoystickRobot' 

	def __init__(self, env, team, bottom_left, angle):
		super().__init__(env, team, bottom_left, angle)
		env.joystick_robot = True
		pygame.joystick.init()
		if pygame.joystick.get_count():
			self.strat = Joystick()	
		else:
			self.strat = DoNothing() 

	def get_strategy(self):
		return self.strat



strats = [DummyRobot, CrazyRobot, AttackRobot]
