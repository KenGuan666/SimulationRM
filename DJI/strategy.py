
from Objects import *
from utils import *
from action import *
from pyglet.window import key
import keyboard

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
                return action + default
            return [action] + default
        return default

    def name():
        pass


class Patrol(Strategy):
    curr_pt_ind = 0 # because apparently this class gets reinstantiated every time its called

    def __init__(self, line_index):
        self.key_points = [
            (Point(600, 350), Point(250, 350)), # top
            (Point(750, 350), Point(750, 100)), # right
        ]
        self.line = self.key_points[line_index]

    def decide(self, robot):
        if float_equals(0, robot.center.dis(self.line[Patrol.curr_pt_ind])):
            Patrol.curr_pt_ind = 1 - Patrol.curr_pt_ind
#        enemy = robot.get_enemy()
#        if robot.center.dis(enemy.center) <= robot.range and \
#           not robot.env.is_blocked(LineSegment(robot.center, enemy.center), [robot, enemy]) and \
#           float_equals(robot.angle_to(enemy.center), robot.angle + robot.gun_angle):
#            return Aim(enemy.center)
        return Move(self.line[Patrol.curr_pt_ind])

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
           robot.env.is_blocked(LineSegment(robot.center, self.target_robot.center), [robot, self.target_robot]):
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


class Manual(Strategy):

    def __init__(self, controls):
        self.controls = controls
        [self.left, self.down, self.right, self.up, self.turnleft, self.turnright, \
            self.fire, self.refill] = controls
        self.shooting = False

    def decide(self, robot):
        if not robot.env.rendering:
            return
        window = robot.env.viewer.window
        actions = []

        @window.event
        def on_key_press(symbol, modifier):
            if symbol == key.B:
                if robot.shooting:
                    SwitchShootingOff().resolve(robot)
                else:
                    SwitchShootingOn().resolve(robot)
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
