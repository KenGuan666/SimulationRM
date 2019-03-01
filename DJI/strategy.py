
from Objects import *
from utils import *
from action import *
from pyglet.window import key
import keyboard
import pyglet
import pygame

"""
All strategies shall implement the abstract class Strategy

A strategy takes in an env state and returns an action

Note that users cannot create new instances of some Strategies.
They use the CLASS OBJECT for operations
"""
class Strategy:

    def decide_with_default(self, robot):
        action = self.decide(robot)
        default = [AutoAim(robot.get_enemy()), AutoShootingControl()]
        if robot.shooting == True:
            default += [Fire()]
        if action:
            if type(action) == list:
                return default + action
            return default + [action]
        return default

    def name():
        pass


class Patrol(Strategy):

    key_points = [Point(0, 0)]

    def decide(self, robot):
        pass

    def name():
        return "PATROL"


class DoNothing(Strategy):

    def decide(self, robot):
        return None

    def name():
        return "DO NOTHING"


class SpinAndFire(Strategy):

    def decide(self, robot):
        # line_block = robot.env.is_blocked(robot.fire_line(), [robot])
        # if line_block:
        #     if line_block.type == "ROBOT" and not (robot.team is line_block.team) or \
        #        line_block.type == "ARMOR" and not (robot.team is line_block.master.team):
        #         return []
        pass

    def name():
        return "SPIN&FIRE"


class Chase(Strategy):

    def __init__(self, target_robot):
        self.target_robot = target_robot

    def decide(self, robot):
        if robot.center.dis(self.target_robot.center) > robot.range or \
           robot.env.is_blocked(LineSegment(robot.center, self.target_robot.center), ignore=[robot, self.target_robot]):
            return [Move(self.target_robot.center)]
        if float_equals(robot.angle_to(self.target_robot.center), robot.angle + robot.gun_angle):
            return None
        return [Aim(self.target_robot.center)]


class Attack(Strategy):

    def decide(self, robot):
        enemy = robot.get_enemy()
        loader = robot.team.loading_zone
        if loader.aligned(robot):
            if loader.loading():
                return None
            if robot.bullet < robot.bullet_capacity - 100 and loader.fills > 0:
                return RefillCommand()
        if robot.bullet > 0:
            return Chase(enemy).decide(robot)
        else:
            return Move(loader.loading_point)


class AttackWithR(Strategy):
    pass


class KeyboardPyglet(Strategy):

    def __init__(self, controls):
        self.controls = controls
        [self.left, self.down, self.right, self.up, self.turnleft, self.turnright, \
            self.refill] = controls
        self.shooting = False

    def decide(self, robot):
        if not robot.env.rendering:
            return
        window = robot.env.viewer.window
        actions = []

        @window.event
        def on_key_press(symbol, modifier):
            if symbol == key.R:
                RefillCommand().resolve(robot)

        if keyboard.is_pressed(self.turnleft):
            actions.append(RotateLeft(robot.max_rotation_speed))
        if keyboard.is_pressed(self.turnright):
            actions.append(RotateRight(robot.max_rotation_speed))
        if keyboard.is_pressed(self.up):
            actions.append(MoveForward(robot.angle, robot.max_speed))
        if keyboard.is_pressed(self.down):
            actions.append(MoveBackward(robot.angle, robot.max_speed))
        if keyboard.is_pressed(self.left):
            actions.append(MoveLeft(robot.angle, robot.max_speed))
        if keyboard.is_pressed(self.right):
            actions.append(MoveRight(robot.angle, robot.max_speed))
        return actions


class KeyboardPygame(Strategy):
    
    def __init__(self, controls):
        [self.left, self.down, self.right, self.up, self.turnleft, self.turnright, \
            self.refill] = controls
        self.actions = []
        self.refilling = False

    def set_instructions(self, actions):
        self.actions = actions
        if self.refilling and self.refill not in actions:
            self.refilling = False

    def decide(self, robot):
        if not self.actions:
            return None
        actions = []
        if self.turnleft in self.actions:
            actions.append(RotateLeft(robot.max_rotation_speed))
        if self.turnright in self.actions:
            actions.append(RotateRight(robot.max_rotation_speed))
        if self.up in self.actions:
            actions.append(MoveForward(robot.angle, robot.max_speed))
        if self.down in self.actions:
            actions.append(MoveBackward(robot.angle, robot.max_speed))
        if self.left in self.actions:
            actions.append(MoveLeft(robot.angle, robot.max_speed))
        if self.right in self.actions:
            actions.append(MoveRight(robot.angle, robot.max_speed))
        if self.refill in self.actions:
            if not self.refilling:
                self.refilling = True
                actions.append(RefillCommand())
        return actions

class Joystick(Strategy):

    refilled = False
        
    def decide(self, robot):
        j = pygame.joystick.Joystick(0)
        j.init()
        coords = [j.get_axis(0), j.get_axis(1), j.get_axis(3)]
        hat = j.get_hat(0)
        
        if not any(hat):
            self.refilled = False

        for i in range(len(coords)):
            if float_equals(coords[i], 0, 0.05):
                coords[i] = 0     
        if not any(coords) and not any(hat):
            return None
        
        [right, down, clockwise] = coords
        translation_power, rotation_power = max(abs(right), abs(down)), abs(clockwise)

        if translation_power:
            translation_weight = translation_power / (translation_power + rotation_power)
            rotation_weight = 1 - translation_weight
            helper_weight = (right ** 2 + down ** 2) ** 0.5
            right_weight, down_weight = right / helper_weight, down / helper_weight
        else:
            rotation_weight = 1

        actions = []
        if right:
            actions.append(MoveRight(robot.angle, robot.max_speed * translation_weight * translation_power * right_weight))
        if down:
            actions.append(MoveBackward(robot.angle, robot.max_speed * translation_weight * translation_power * down_weight))
        if clockwise:
            actions.append(RotateRight(robot.max_rotation_speed * rotation_weight * clockwise))
        if any(hat) and not self.refilled:
            self.refilled = True
            actions.append(RefillCommand())

        # translation_angle = (Point(0, 0).angle_to(Point(right, -down)) - 90) % 360
        # print(translation_angle)
        # action = MoveAtAngle(robot.angle, robot.max_speed * translation_weight * translation_power, translation_angle)
        return actions