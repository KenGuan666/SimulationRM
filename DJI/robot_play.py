import gym
import numpy as np
import time
import cv2
import pygame
from robomaster import *
from utils import *

env = gym.make('Robomaster-v0').unwrapped
total_rounds = int(env.full_time / env.tau)
joystick_control = True

#TODO note that yaw translation might be wrong

#transforms x,y ROS -> Sim // given that ROS gives origin top right, Sim gives origin bottom left
def ros_to_sim_x(x):
	return 800. - (x * 100.)
def ros_to_sim_y(y):
	return 500. - (y * 100.)
def ros_to_sim_yaw(yaw):
	return (to_degree(yaw) + 180.) % 360 #given as radian, convert to degree, flip 180

def sim_to_ros_x(x):
	return (800. - x) / 100.
def sim_to_ros_y(y):
	return (500. - y) / 100.
def sim_to_ros_yaw(yaw):
	return to_radian((yaw + 180) % 360) #given degree, flip 180, convert to degree

# new options
env.ros_control = True

for _ in range(total_rounds):
	env.step()
	if env.ready_to_publish():
		#publish path logic here
		path = env.compute_path_to_publish()
		print(path)
	if env.finished:
		break