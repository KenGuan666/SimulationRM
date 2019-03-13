
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

    def resolve(self, robot): #TODO resolve bug, sometimes doesn't shoot when it can
        if robot.aimed_at_enemy():
            robot.shooting = True
        else:
            robot.shooting = False

class AutoRotate(Action):

    def __init__(self, target_robot, degrees_offset=45):
        self.target = target_robot
        self.degrees_offset = degrees_offset

    def resolve(self, robot):
        ls = LineSegment(robot.center, self.target.center)
        degrees = (math.degrees(math.atan2(ls.y_diff, ls.x_diff)) + self.degrees_offset) % 360
        return Rotate(degrees).resolve(robot)

#TODO figure out where to put 'limit' variable -> it controls after how many ticks we recompute the path
class Move(Action):
    """
    IMPORTANT: will remember path computed for MOVE
    to reset the path computed, either counter must == conter_max
    OR self.path must be set to None
    """

    ticks_until_astar_recalc = None #Should get overridden in robotmaster.py

    def __init__(self, target_point):
        self.target_point = target_point # If target_point is None, then do nothing
        self.counter_max = Move.ticks_until_astar_recalc
        self.counter = Move.ticks_until_astar_recalc
        self.path = None

    def set_target_point(self, target_point, recompute, force_compute=False):

        if force_compute or (recompute and not self.target_point.float_equals(target_point)):
            self.path = None
        self.target_point = target_point

    def resolve(self, robot):
        if self.target_point is None or robot.center.float_equals(self.target_point):
            self.path = None
            return

        #check if need to recalc path
        self.counter += 1
        if self.counter >= self.counter_max:
            self.counter = 0
            self.path = full_astar(self.target_point, robot)

        # init path
        if not self.path:
            self.path = full_astar(self.target_point, robot)
        #update waypoint
        if robot.center.float_equals(self.path[0]):
            if len(self.path) > 2 or robot.env.direct_reachable_curr_angle(robot.center, self.path[-1], robot):
                self.path = self.path[1:]

        #move to first waypoint of path
        if (len(self.path) > 0) and robot.env.direct_reachable_curr_angle(robot.center, self.path[0], robot):
            return MoveAtAngle(0, min(robot.center.dis(self.path[0]),
                   robot.max_speed), robot.angle_to(self.path[0])).resolve(robot)

        # WAITING ON BETTER PATH ALGORITHM


## PATH ALGORITHM GOES HERE FOR NOW
# returns path
def full_astar(to, robot):
    env = robot.env
    master = env.master_network
    removed_weighted_edges = []
    fr_id = len(env.network_points)
    to_id = fr_id + 1
    master.add_nodes_from([fr_id, fr_id + 1])
    # TODO make sure extra_edges are found mathematically not hard coded
    # master.add_weighted_edges_from([robot.team.extra_weighted_edge])
    points = env.network_points + [robot.center, to]
    closest_point, min_dis = None, 9999

    extra_ignore = []
    for r in env.characters['robots']:
        if r.center.float_equals(to):
            extra_ignore = [r]
            break

    # remove illegal edges
    for r in env.characters['robots']:
        if r is robot or r in extra_ignore:
            continue
        points_in_radius = get_network_points(env.network_points, r) #TODO make parameter for illegal edge collision
        for p_i in points_in_radius:
            for j_id in list(master.adj[p_i.id]):
                p_j = points[j_id]
                if master.has_edge(p_i, p_j) and not r.blocks_path_curr_angle(p_i, p_j, robot):
                    removed_weighted_edges.append((p_i.id, p_j.id, master[p_i.id][p_j.id]['weight'])) # save weight

    # for all nodes visible to robot.center and to-point, add an edge + weight
    for p in env.network_points:
        if p.dis(robot.center) <= 250 and env.direct_reachable_curr_angle(robot.center, p, robot): #TODO make parameter for start/goal visibility
            master.add_edge(fr_id, p.id, weight=robot.center.dis(p))
        dis = p.dis(to)
        if dis and dis < min_dis:
            closest_point, min_dis = p, dis
        if p.dis(to) <= 250 and env.direct_reachable_curr_angle(p, to, robot, extra_ignore=extra_ignore):
            master.add_edge(p.id, to_id, weight=dis)

    # finally checks for a visible edge between robot.center and to-point
    if env.direct_reachable_curr_angle(robot.center, to, robot, extra_ignore=extra_ignore):
        master.add_edge(fr_id, to_id, weight=robot.center.dis(to))

    # remove illegal edges
    for e in removed_weighted_edges:
        try:
            master.remove_edge(e[0], e[1])
        except nx.NetworkXError as e:
            continue

    # search for astar path, if path not found, move to the closest point on the graph to the to-point
    try:
        path = nx.astar_path(master, fr_id, to_id)
    except nx.NetworkXNoPath as e:
        master.add_weighted_edges_from(removed_weighted_edges)
        removed_weighted_edges = []
        # master.remove_edge(robot.team.extra_weighted_edge[0], robot.team.extra_weighted_edge[1]) #TODO
        master.remove_node(fr_id)
        master.remove_node(to_id)
        if closest_point:
            return full_astar(closest_point, robot)
        return None

    # display options
    # display_edges(points, env) # renders the WHOLE graph
    if env.rendering:
        display_path(path, points, to, env) # renders the planned path

    master.add_weighted_edges_from(removed_weighted_edges)
    # master.remove_edge(robot.team.extra_weighted_edge[0], robot.team.extra_weighted_edge[1]) #TODO
    master.remove_node(fr_id)
    master.remove_node(to_id)
    return [points[path[i]] for i in range(1,len(path))] # remove current robot point


def get_network_points(points, robot):
    env = robot.env
    center = robot.center
    return [p for p in points if p.dis(center) < 140]
   

