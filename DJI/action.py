
from Objects import *
from utils import *

"""
An Action is a command to the robot

It changes the state of the robot upon resolve
"""
class Action:

    # Returns True if the Action is successfully resolved
    # Returns False if it didn't have any effect
    def resolve(self, robot):
        result_rec = self.simple_resolve(robot)
        if not result_rec:
            self.post_resolve(robot)
            return True
        if robot.env.isObstructed(result_rec, robot):
        	return False

        robot.setPosition(result_rec)
        self.post_resolve(robot)
        return True

    def post_resolve(self, robot):
        pass


class Step(Action):

    def __init__(self, robot_angle, dis):
        self.angle += robot_angle
        self.dx = math.cos(toRadian(self.angle)) * dis
        self.dy = math.sin(toRadian(self.angle)) * dis

    def simple_resolve(self, robot):
        return Rectangle(robot.bottom_left.move(self.dx, self.dy), robot.width, robot.height, robot.angle)


class StepForward(Step):
    angle = 0

class StepBackward(Step):
    angle = 180

class StepLeft(Step):
    angle = 90

class StepRight(Step):
    angle = 270


class Rotate(Action):

    def __init__(self, angle):
        self.angle = angle

    def simple_resolve(self, robot):
        if floatEquals(robot.angle, self.angle):
        	return
        diff = self.angle - robot.angle
        min_dis = min(diff, diff + 360, diff - 360, key = lambda n: abs(n))
        if min_dis > 0:
            self.final_angle = min(robot.max_rotation_speed, min_dis)
            self.dir = "LEFT"
            action = RotateLeft(self.final_angle)
        else:
            self.final_angle = min(robot.max_rotation_speed, -min_dis)
            self.dir = "RIGHT"
            action = RotateRight(self.final_angle)

        return action.simple_resolve(robot)

    def post_resolve(self, robot):
        if self.dir == "LEFT":
            return RotateGunRight(self.final_angle).resolve(robot)
        return RotateGunLeft(self.final_angle).resolve(robot)



class RotateLeft(Action):

    def __init__(self, angle):
        self.angle = angle

    def simple_resolve(self, robot):
        return Rectangle.by_center(robot.center, robot.width, robot.height, robot.angle + self.angle)


class RotateRight(Action):

    def __init__(self, angle):
        self.angle = angle

    def simple_resolve(self, robot):
        return Rectangle.by_center(robot.center, robot.width, robot.height, robot.angle - self.angle)


class RefillCommand(Action):

    def resolve(self, robot):
        print(robot.team.name + " team issued reload command!")
        robot.team.loadingZone.load(robot)


class RotateGunLeft(Action):

    def __init__(self, angle):
        self.angle = angle

    def resolve(self, robot):
        robot.gun_angle = min(robot.gun_angle + self.angle, robot.max_gun_angle)


class RotateGunRight(Action):

    def __init__(self, angle):
        self.angle = angle

    def resolve(self, robot):
        robot.gun_angle = max(robot.gun_angle - self.angle, -robot.max_gun_angle)


class AutoAim(Action):

    def __init__(self, target_robot):
        self.target = target_robot

    def resolve(self, robot):
        goal, curr = robot.angleTo(self.target.center) - robot.angle, robot.gun_angle
        if floatEquals(goal, curr):
        	return
        diff = goal - curr
        if diff > 180:
            diff -= 360
        if diff < -180:
            diff += 360
        if diff > 0:
            angle = min(robot.max_gun_rotation_speed, diff)
            action = RotateGunLeft(angle)
        else:
            angle = min(robot.max_gun_rotation_speed, -diff)
            action = RotateGunRight(angle)

        return action.resolve(robot)


class Aim(Action):

    def __init__(self, target_point):
        self.target_point = target_point

    def resolve(self, robot):
        return Rotate(robot.angleTo(self.target_point)).resolve(robot)


class Fire(Action):

    def resolve(self, robot):
        if robot.bullet > 0 and robot.cooldown == 0:
            robot.env.characters['bullets'].append(Bullet(robot.getGun().center, robot.angle + robot.gun_angle, robot.env))
            robot.bullet -= 1
            robot.cooldown = robot.max_cooldown


class SwitchShootingOn(Action):

    def resolve(self, robot):
        robot.shooting = True


class SwitchShootingOff(Action):

    def resolve(self, robot):
        robot.shooting = False


class Move(Action):

    def __init__(self, target_point):
        self.target_point = target_point

    def resolve(self, robot):
        if robot.center.floatEquals(self.target_point):
            return
        if floatEquals(robot.angleTo(self.target_point), robot.angle):
            return StepForward(robot.angle, min(robot.center.dis(self.target_point), \
                robot.max_forward_speed)).resolve(robot)
        return Aim(self.target_point).resolve(robot)
