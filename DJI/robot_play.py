import gym
import numpy as np
import time
import cv2
import pygame
from robomaster import *
from utils import *

env = gym.make('Robomaster-v0').unwrapped
# env.init_from_state([1058, 1, 408.6590398623192, 415.1700916785658, 68.14982363927881, 90, 73, 0, 0, 0, 1800, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 238.0520211100148, 79.06723937912398, 112.19999999999996, -50.713777541139656, 24, 0, 0, 0, 1650, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 26.799999999999628, 73.19999999999996, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])

total_rounds = int(env.full_time / env.tau)
joystick_control = True
# clock = pygame.time.Clock()
# pygame.joystick.init()

for _ in range(total_rounds):

	# for e in pygame.event.get():
	# 	if e.type == pygame.QUIT:
	# 		# pygame.quit()
	# 		env.close()

	# j = pygame.joystick.Joystick(0)
	# j.init()

	# coords = [j.get_axis(0), j.get_axis(1), j.get_axis(3)]

	# for i in range(len(coords)):
	# 	if float_equals(coords[i], 0, 0.05):
	# 		coords[i] = 0

	# env.coords = coords
	# print(coords)
	# clock.tick(1)
	env.step()

	# time.sleep(0.01)
	if env.finished:
		break