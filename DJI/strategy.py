from abc import abstractmethod

import keyboard
import pygame
from pyglet.window import key

from Objects import *
from action import *

"""
All strategies shall implement the abstract class Strategy

A strategy takes in an env state and returns an action

Note that users cannot create new instances of some Strategies.
They use the CLASS OBJECT for operations
"""
class Strategy:

    def __init__(self):
        self.move = Move(None) #None means there is no where  move to, .resolve will do nothing
        self.auto_aim = True
        self.auto_shoot = True
        self.auto_rotate = True

    def move_to(self, target_point, recompute, force_compute = False, backups = []):
        """
        :param target_point: point to move to
        :param recompute: whether or not to immediately recalculate A* IF target_point changes,
                            false -> keep following old path
                            true && target changes-> find new path (does nothing if target_point is null)
        :param force_compute: if true, always recompute path, even if target_point is the same
        """
        if target_point:
            self.move.set_target_point(target_point, recompute, force_compute=force_compute, backups=backups)

    def force_path_recompute(self):
        self.move.path = None

    def stay_put(self):
        """
        tells the move object to NOT move
        """
        self.move_to(None, recompute=True)

    def substrat_decide(self, substrat, robot):
        temp = substrat.decide(robot)
        self.move = substrat.move
        return temp

    def set_substrat(self, substrat):
        substrat.move = self.move

    def decide_with_default(self, robot):
        action = self.decide(robot)
        enemy = robot.get_enemy()
        default = []
        #AUTO ROTATE
        if self.auto_aim:
            default.append(AutoAim(enemy))
        if self.auto_shoot:
            default.append(AutoShootingControl())
        if self.auto_rotate:
            default.append(AutoRotate(enemy, degrees_offset=45))
        if robot.shooting == True:
            default.append(Fire())
        if action:
            if type(action) == list:
                return default + action
            return default + [action]
        return default + [self.move]

    @abstractmethod
    def decide(self, robot):
        pass

    @classmethod
    def name(cls):
        return cls.__name__


class Patrol(Strategy):

    key_points = [Point(0, 0)]

    def decide(self, robot):
        pass


class DoNothing(Strategy):

    def decide(self, robot):
        return None


class BreakLine(Strategy):

    def __init__(self):
        Strategy.__init__(self)
        self.additional_key_points = []

    def decide(self, robot):
        enemy = robot.get_enemy()
        if robot.center.dis(enemy.center) > robot.range or \
           robot.env.is_blocked(LineSegment(robot.center, enemy.center), [robot, enemy]):
            # print('blocked!')
            return None
        else:
            # print('not blocked!')
            search_pts = sorted(robot.env.network_points + self.additional_key_points, key=lambda x: robot.center.dis(x))
            for i in search_pts:
                if robot.env.is_blocked(LineSegment(enemy.center, i), [robot, enemy]):
                    loader = robot.team.loading_zone
                    self.move_to(i, recompute=True)
                    return None


class SpinAndFire(Strategy):

    def decide(self, robot):
        self.auto_rotate = False
        return RotateRight(robot.max_rotation_speed)


class Chase(Strategy):

    def __init__(self):
        Strategy.__init__(self)
        self.target_robot = None

    def choose_target_robot(self, target_robot):
        self.target_robot = target_robot

    def decide(self, robot):
        if self.target_robot is None:
            self.choose_target_robot(robot.get_enemy())
        if robot.center.dis(self.target_robot.center) > robot.range or \
           robot.env.is_blocked(LineSegment(robot.get_gun_base(), self.target_robot.center), ignore=[robot, self.target_robot]):
            self.move_to(self.target_robot.center, False) # don't need an immediate recompute for chasing
            return None
        if float_equals(robot.angle_to(self.target_robot.center), robot.angle + robot.gun_angle):
            self.stay_put()
            return None
        return [Aim(self.target_robot.center)]


class OnlyReload(Strategy):

    def decide(self, robot):
        loader = robot.team.loading_zone
        if loader.aligned(robot):
            if loader.loading():
                return None
            if robot.bullet < robot.bullet_capacity and loader.fills > 0:
                return RefillCommand()
        else:
            return self.move_to(loader.loading_point, recompute=False)


class GetDefenseBuff(Strategy):

    def decide(self, robot):
        buff_pts = [z.center for z in robot.env.defense_buff_zones]
        buff_pts = sorted(buff_pts, key=lambda x: robot.center.dis(x))
        return self.move_to(buff_pts[0], recompute=True, backups = [buff_pts[1]])


class Attack(Strategy):

    def __init__(self):
        Strategy.__init__(self)
        self.chase_sub_strat = Chase()
        self.set_substrat(self.chase_sub_strat)
        self.chase_sub_strat.move.print_bool = True #TODO REMOVE

    def decide(self, robot):
        enemy = robot.get_enemy()
        loader = robot.team.loading_zone
        if loader.aligned(robot):
            if loader.loading():
                return None
            if robot.bullet < robot.bullet_capacity - 100 and loader.fills > 0:
                return RefillCommand()
        if robot.bullet > 0:
            self.chase_sub_strat.choose_target_robot(enemy)
            # return Chase(enemy).decide(robot)
            return self.substrat_decide(self.chase_sub_strat, robot)
        else:
            return self.move_to(loader.loading_point, recompute=True)
            # return Move(loader.loading_point)


class AttackWithR(Strategy):
    pass


class KeyboardPyglet(Strategy):

    def __init__(self, controls, ignore_angle):
        Strategy.__init__(self)
        self.controls = controls
        [self.left, self.down, self.right, self.up, self.turnleft, self.turnright,
            self.refill] = controls
        self.shooting = False
        self.ignore_angle = ignore_angle

    def decide(self, robot):
        board_ang = 90
        if not robot.env.rendering:
            return
        window = robot.env.viewer.window
        actions = []

        @window.event
        def on_key_press(symbol, modifier):
            if symbol == key.R:
                if robot.team.loading_zone.aligned(robot):
                    RefillCommand().resolve(robot)
                else:
                    Move(robot.team.loading_zone.center).resolve(robot)

        if keyboard.is_pressed(self.turnleft):
            actions.append(RotateLeft(robot.max_rotation_speed))
        if keyboard.is_pressed(self.turnright):
            actions.append(RotateRight(robot.max_rotation_speed))
        if keyboard.is_pressed(self.up):
            actions.append(MoveForward(board_ang if self.ignore_angle else robot.angle, robot.max_speed))
        if keyboard.is_pressed(self.down):
            actions.append(MoveBackward(board_ang if self.ignore_angle else robot.angle, robot.max_speed))
        if keyboard.is_pressed(self.left):
            actions.append(MoveLeft(board_ang if self.ignore_angle else robot.angle, robot.max_speed))
        if keyboard.is_pressed(self.right):
            actions.append(MoveRight(board_ang if self.ignore_angle else robot.angle, robot.max_speed))
        return actions


class KeyboardPygame(Strategy):
    
    def __init__(self, controls, ignore_angle):
        Strategy.__init__(self)
        [self.left, self.down, self.right, self.up, self.turnleft, self.turnright, \
            self.refill] = controls
        self.actions = []
        self.refilling = False
        self.ignore_angle = ignore_angle

    def set_instructions(self, actions):
        self.actions = actions
        if self.refilling and self.refill not in actions:
            self.refilling = False

    def decide(self, robot):
        board_ang = 90
        if not self.actions:
            return None
        actions = []
        if self.turnleft in self.actions:
            actions.append(RotateLeft(robot.max_rotation_speed))
        if self.turnright in self.actions:
            actions.append(RotateRight(robot.max_rotation_speed))
        if self.up in self.actions:
            actions.append(MoveForward(board_ang if self.ignore_angle else robot.angle, robot.max_speed))
        if self.down in self.actions:
            actions.append(MoveBackward(board_ang if self.ignore_angle else robot.angle, robot.max_speed))
        if self.left in self.actions:
            actions.append(MoveLeft(board_ang if self.ignore_angle else robot.angle, robot.max_speed))
        if self.right in self.actions:
            actions.append(MoveRight(board_ang if self.ignore_angle else robot.angle, robot.max_speed))
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
