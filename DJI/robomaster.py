"""
Environment for DJI Robomaster AI Challenge.
"""

import math
import gym
from gym import spaces, logger
from utils import *
#from gym.envs.classic_control import rendering
import rendering
from gym.utils import seeding
from gym.envs.DJI.Objects import *
import numpy as np
import cv2

"""
Currently doesn't fit into the OpenAI gym model

For future modification, consider definition of action set
"""
class RobomasterEnv(gym.Env):
	# """
	# Description:
	# 	A robot placed in a battlefield to face off against an enemy robot. Robots fire at each other to deal damage.
	# 	Score is calculated at the end of a game to determine the winner.
	#
	# Source:
	# 	This environment is a simplified version of the DJI Robomaster AI Challenge field.
	#
	# Observation:
	# 	Type: Box(4)
	# 	Num 	Observation 			Min 		Max
	# 	0		Robot X Position		0.0			800.0
	# 	1		Robot Y Position		0.0			500.0
	# 	2		Enemy X Position		0.0			800.0
	# 	3		Enemy Y Position		0.0			500.0
	#
	# Actions:
	# 	Type: Box(2)
	# 	Num 	Action 			Min			Max
	# 	0		X Position		0			800
	# 	1		Y Position		0			500
	#
	# Reward:
	# 	Reward is 1 for every shot taken at the enemy.
	# 	Reward is -1 for every shot taken from the enemy.
	#
	# Starting State:
	# 	Initial Robot X and Robot Y are both 0.0.
	# 	Initial Enemy X and Enemy Y are 800.0 and 500.0 respectively.
	#
	# Episode Termination:
	# 	Robot health is 0.
	# 	Enemy health is 0.
	# 	Episode length is greater than 300.
	# """
	#
	# metadata = {
	# 	'render.modes': ['human', 'rgb_array'],
	# 	'video.frames_per_second': 50
	# }

	# Defining course dimensions
	width = 800
	height = 500
	tau = 0.02
	full_time = 300
	display_visibility_map = False

	def __init__(self):

		# Record time
		self.game_time = 0
		self.finished = False

		self.characters = {
			"obstacles": [],
			"zones": [],
			"robots": [],
			"bullets": [],
		}

		# Initialize teams
		BLUE = Team("BLUE")
		RED = Team("RED")
		BLUE.enemy, RED.enemy = RED, BLUE
		self.my_team, self.enemy_team = BLUE, RED

		# Initialize robots
		my_robot = AttackRobot(self, BLUE, Point(170, 295), 0)
		enemy_robot = ManualControlRobot("OSPWADBR", self, RED, Point(10, 10), 0)
		my_robot.load(40)
		enemy_robot.load(40)
		self.characters['robots'] = [my_robot, enemy_robot]
		for i in range(len(self.characters['robots'])):
			self.characters['robots'][i].id = i

		# Defining course obstacles
		self.characters['obstacles'] = [Obstacle(p[0], p[1], p[2])
		    for p in [(Point(325, 0), 25, 100), (Point(450, 400), 25, 100),
				(Point(350, 238), 100, 25), (Point(580, 100), 100, 25),
				(Point(120, 375), 100, 25), (Point(140, 140), 25, 100),
				(Point(635, 260), 25, 100)]]

		# Team start areas
		self.starting_zones = [StartingZone(p[0], p[1]) for p in [(Point(0, 0), BLUE),
		(Point(700, 0), BLUE), (Point(0, 400), RED), (Point(700, 400), RED)]]

		self.defense_buff_zones = [DefenseBuffZone(p[0], p[1], self) for p in [
		(Point(120, 275), BLUE), (Point(580, 125), RED)]]

		self.loading_zones = [LoadingZone(p[0], p[1]) for p in [
		(Point(350, 0), RED), (Point(350, 400), BLUE)]]

		self.characters['zones'] = self.starting_zones + self.defense_buff_zones + \
		    self.loading_zones

		# self.background = Rectangle(Point(0, 0), self.width, self.height, 0)

		self.viewer = rendering.Viewer(self.width, self.height)

		boundary = rendering.PolyLine([(1, 0), (1, 499), (800, 499), (800, 0)], True)
		self.viewer.add_geom(boundary)

		health_bar_params = (20, 260)
		BLUE.set_health_bar(UprightRectangle(Point(10, self.height / 2 - health_bar_params[1] / 2), \
		    health_bar_params[0], health_bar_params[1]), self.viewer)
		RED.set_health_bar(UprightRectangle(Point(self.width - health_bar_params[0] - 10, \
		    self.height / 2 - health_bar_params[1] / 2), health_bar_params[0], health_bar_params[1]), self.viewer)

		for char in self.inactables():
			geoms = char.render()
			for geom in geoms:
				self.viewer.add_geom(geom)


		# Init movement network
		delta = 25
		network_points = []
		for block in self.characters['obstacles'] + [self.my_team.loading_zone]:
			for i in range(4):
				delta_x, delta_y = abs(i - 1.5) // 1.5 * 2 - 1, i // 2 * 2 - 1
				delta_x *= -delta
				delta_y *= delta
				point = block.vertices[i].move(delta_x, delta_y)
				if self.is_legal(point):
					network_points.append(point)
					if self.display_visibility_map:
						geom = rendering.Circle(point, 5)
						self.viewer.add_geom(geom)

		network_edges = []
		for i in range(len(network_points)):
			for j in range(i + 1, len(network_points)):
				p_i, p_j = network_points[i], network_points[j]
				if self.direct_reachable_forward(p_i, p_j, my_robot):
					network_edges.append(LineSegment(p_i, p_j))
					if self.display_visibility_map:
						edge = rendering.PolyLine([p_i.to_list(), p_j.to_list()], False)
						self.viewer.add_geom(edge)

	def state(self):
		return []

	def actables(self):
		return self.loading_zones + self.defense_buff_zones + \
		    self.characters['robots'] + self.characters['bullets']

	def inactables(self):
		return self.starting_zones + self.characters['obstacles']

	def unpenetrables(self):
		plates = []
		for r in self.characters['robots']:
			plates += r.get_armor()
		return plates + self.characters['robots'] + self.characters['obstacles']

	def impermissibles(self, robot):
		return list(filter(lambda r: not r == robot, self.characters['robots'])) \
		    + self.characters['obstacles'] \
		    + list(filter(lambda z: not z.permissble(robot.team), self.loading_zones))

	def direct_reachable_forward(self, fr, to, robot):
		if fr.float_equals(to):
			return True
		helper_robot = Rectangle.by_center(fr, robot.width, robot.height, fr.angle_to(to))
		helper_rec = Rectangle(helper_robot.bottom_left, fr.dis(to) + robot.width, \
		    robot.height, fr.angle_to(to))
		return not self.is_obstructed(helper_rec, robot)

	# def direct_reachable_sideways

	def has_winner(self):
		my_health, enemy_health = self.my_team.total_health(), self.enemy_team.total_health()
		if my_health == 0:
			return self.enemy_team
		if enemy_health == 0:
			return self.my_team
		if self.game_time == self.full_time:
			if my_health > enemy_health:
				return self.my_team
			if enemy_health > my_health:
				return self.enemy_team
			return "DRAW"

	# Moves the game 1 timestep defined by self.tau
	def step(self):
		# """
		# robot_action: a point(x, y) the robot will travel to
		# enemy_action: a point(x, y) the enemy robot will travel to
		# """

		winner = self.has_winner()
		if winner:
			self.finished = True
			if winner == "DRAW":
				print('GAME OVER. RESULT: DRAW')
			else:
				print('GAME OVER. WINNER IS {0}.'.format(winner.name))
			self.close()
			return

		self.game_time += self.tau

		if int(self.game_time) % 30 == 0:
			for z in self.defense_buff_zones + self.loading_zones:
				z.reset()

		for char in self.actables():
			char.act()

		# NOT YET IMPLEMENTED
		reward = 0.0

		# NOT YET IMPLEMENTED
		done = False

		return np.array(self.state), reward, done, {}

	# Resets the field to the starting positions
	def reset(self):
		self.viewer.close()
		self.__init__()
		return np.array(self.state)

	# Renders the field for human observation
	def render(self, mode='human'):

		for char in self.actables():
			geoms = char.render()
			for geom in geoms:
				self.viewer.add_onetime(geom)
		self.viewer.add_onetime_text("Time: {0} seconds".format(round(self.game_time, 1)), \
		    10, 12, self.height - 12)

		return self.viewer.render(return_rgb_array = mode == 'rgb_array')

	def is_obstructed(self, rec, robot):
		for ob in self.impermissibles(robot):
			if ob.intersects(rec):
				return True
		for v in rec.vertices:
			if not self.is_legal(v):
				return True
		return False

	# Returns the object blocking the line segment
	# Returns False if it's not blocked
	def is_blocked(self, seg, ignore=[]):
		for block in self.unpenetrables():
			if block.blocks(seg) and not block in ignore:
				return block
		return False

	def is_legal(self, point):
		return point.x >= 0 and point.x <= self.width and point.y >= 0 and point.y <= self.height

	def close(self):
		if self.viewer:
			self.viewer.close()
			self.viewer = None
