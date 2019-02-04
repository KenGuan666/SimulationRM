
from Objects import *
from utils import *
from action import *
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
        default = [AutoAim(robot.getEnemy())]
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
        actions = []
        if robot.angle % 90 == 0:
            actions.append(SwitchShootingOn())
        if robot.angle < 180:
            return actions + [Rotate(180)]
        if robot.angle >= 180 and robot.angle < 270:
            return actions + [Rotate(270)]
        return actions + [Rotate(0)]

    def name():
        return "SPIN&FIRE"


class AimAndFire(Strategy):

    def __init__(self, target_robot):
        self.target_robot = target_robot

    def decide(self, robot):
        if floatEquals(robot.angleTo(self.target_robot.center), robot.angle + robot.gun_angle):
            fire_line = LineSegment(robot.getGun().center, self.target_robot.center)
            if robot.env.isBlocked(fire_line, self.target_robot):
                return
            if fire_line.length() > robot.range:
                return Move(self.target_robot.center)
            return SwitchShootingOn()
        return [SwitchShootingOff(), Aim(self.target_robot.center)]


class Attack(Strategy):

    def decide(self, robot):
        enemy = robot.getEnemy()
        loader = robot.team.loadingZone
        if robot.bullet > 0:
            return AimAndFire(enemy).decide(robot)
        elif loader.aligned(robot):
            if loader.fills > 0:
                return RefillCommand()
            return None
        else:
            return Move(loader.loadingPoint)


class Manual(Strategy):

    def __init__(self, controls):
        self.controls = controls
        [self.left, self.down, self.right, self.up, self.turnleft, self.turnright, \
            self.fire, self.refill] = controls

    def decide(self, robot):
        actions = []
        if keyboard.is_pressed(self.turnleft):
            actions.append(RotateLeft(robot.max_rotation_speed))
        if keyboard.is_pressed(self.turnright):
            actions.append(RotateRight(robot.max_rotation_speed))
        if keyboard.is_pressed(self.up):
            actions.append(StepForward(robot.angle, robot.max_forward_speed))
        if keyboard.is_pressed(self.down):
            actions.append(StepBackward(robot.angle, robot.max_forward_speed))
        if keyboard.is_pressed(self.left):
            actions.append(StepLeft(robot.angle, robot.max_forward_speed))
        if keyboard.is_pressed(self.right):
            actions.append(StepRight(robot.angle, robot.max_forward_speed))
        if keyboard.is_pressed(self.fire):
            if robot.shooting:
                actions.append(SwitchShootingOff())
            else:
                actions.append(SwitchShootingOn())
        if keyboard.is_pressed(self.refill):
            actions.append(RefillCommand())
        return actions
