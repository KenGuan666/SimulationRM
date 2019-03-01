
from Objects import *
from utils import *
import rendering
import pygame
import numpy
import networkx as nx

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
        if robot.env.is_obstructed(result_rec, robot):
        	return False

        robot.set_position(result_rec)
        self.post_resolve(robot)
        return True

    def post_resolve(self, robot):
        pass


class Translation(Action):

    steps = 2

    def __init__(self, robot_angle, dis):
        self.angle += robot_angle
        self.dx = math.cos(to_radian(self.angle)) * dis
        self.dy = math.sin(to_radian(self.angle)) * dis

    def resolve(self, robot):
        single_step = Step(self.angle, self.dx / self.steps, self.dy / self.steps)
        for _ in range(self.steps):
            if not single_step.resolve(robot):
                return

class Step(Action):

    def __init__(self, angle, dx, dy):
        self.angle = angle
        self.dx = dx
        self.dy = dy

    def simple_resolve(self, robot):
        return Rectangle(robot.bottom_left.move(self.dx, self.dy), robot.width, robot.height, robot.angle)


class MoveForward(Translation):
    angle = 0

class MoveBackward(Translation):
    angle = 180

class MoveLeft(Translation):
    angle = 90

class MoveRight(Translation):
    angle = 270

class MoveAtAngle(Translation):

    def __init__(self, robot_angle, dis, angle):
        self.angle = angle
        super().__init__(0, dis)


class Rotate(Action):

    def __init__(self, angle):
        self.angle = angle
        self.dir = None

    def simple_resolve(self, robot):
        if float_equals(robot.angle, self.angle):
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
        if self.dir == "RIGHT":
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
        robot.team.loading_zone.load()


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
        goal, curr = robot.angle_to(self.target.center) - robot.angle, robot.gun_angle
        if float_equals(goal, curr):
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
        return Rotate(robot.angle_to(self.target_point)).resolve(robot)


class Fire(Action):

    def resolve(self, robot):
        if robot.bullet > 0 and robot.cooldown == 0:
            noise = np.random.normal(0, 3)
            robot.env.characters['bullets'].append(Bullet(robot.get_gun().center, \
                robot.angle + robot.gun_angle + noise, robot.env, robot))
            robot.bullet -= 1
            robot.cooldown = robot.max_cooldown


class SwitchShootingOn(Action):

    def resolve(self, robot):
        robot.shooting = True


class SwitchShootingOff(Action):

    def resolve(self, robot):
        robot.shooting = False


class AutoShootingControl(Action):

    def resolve(self, robot):
        if robot.aimed_at_enemy():
            robot.shooting = True
        else:
            robot.shooting = False


class Move(Action):

    def __init__(self, target_point):
        self.target_point = target_point

    def resolve(self, robot):
        if robot.center.float_equals(self.target_point):
            return
        # if robot.env.direct_reachable_forward(robot.center, self.target_point, robot, True):
        #     if float_equals(robot.angle_to(self.target_point), robot.angle):
        #         return MoveForward(robot.angle, min(robot.center.dis(self.target_point), \
        #            robot.max_speed)).resolve(robot)
        #     return Aim(self.target_point).resolve(robot)
        if robot.env.direct_reachable_curr_angle(robot.center, self.target_point, robot):
            return MoveAtAngle(0, min(robot.center.dis(self.target_point), \
                   robot.max_speed), robot.angle_to(self.target_point)).resolve(robot)

        # WAITING ON BETTER PATH ALGORITHM
        return astar_ignore_enemy(self.target_point, robot)


## PATH ALGORITHM GOES HERE FOR NOW

def astar_ignore_enemy(to, robot):
    env = robot.env
    G = env.network
    fr_id = len(env.network_points)
    to_id = fr_id + 1
    G.add_nodes_from([fr_id, fr_id + 1])

    points = env.network_points + [robot.center, to]
    closest_point, min_dis = None, 9999
    removed_point, restore_edges = None, None

    def restore_graph():
        env.network.remove_node(fr_id)
        env.network.remove_node(to_id)
        if removed_point:
            env.network.add_node(removed_point)
            for p_id in restore_edges:
                env.network.add_edge(points[p_id].id, removed_point.id, weight=points[p_id].dis(removed_point))

    # total_edges = []
    for p in env.network_points:
        if p.float_equals(robot.center):
            removed_point = p
            restore_edges = list(G.adj[p.id])
            G.remove_node(p.id)
            continue
        if env.direct_reachable_curr_angle(robot.center, p, robot):
            # total_edges.append(LineSegment(robot.center, p))
            G.add_edge(fr_id, p.id, weight=robot.center.dis(p))
        dis = p.dis(to)
        if dis and dis < min_dis:
            closest_point, min_dis = p, dis
        if env.direct_reachable_forward(p, to, robot, ignore_robots=True):
            # total_edges.append(LineSegment(p, to))
            G.add_edge(p.id, to_id, weight=dis)

    # total_edges += env.network_edges
   
    try:
        path = nx.astar_path(G, fr_id, to_id)
    except nx.NetworkXNoPath as e:
        restore_graph()
        if closest_point:
            return Move(closest_point).resolve(robot)
        return None

    if env.rendering:
        for i in range(len(path) - 1):
            edge = rendering.PolyLine([points[path[i]].to_list(), points[path[i + 1]].to_list()], False)
            env.viewer.add_onetime(edge)
        edge = rendering.PolyLine([points[path[len(path) - 1]].to_list(), to.to_list()], False)
        env.viewer.add_onetime(edge)

    # if points[path[1]].float_equals(robot.center):
    #     return Move(points[path[2]]).resolve(robot)
    restore_graph()
    return Move(points[path[1]]).resolve(robot)

    # for p in points:
    #     geom = rendering.Circle(p, 5)
    #     env.viewer.add_onetime(geom)
