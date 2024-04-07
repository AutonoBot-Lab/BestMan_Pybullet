"""
@Description :   A few functions intergrating between robot and ompl
@Author      :   Yan Ding 
@Time        :   2023/08/30 23:00:27
"""

import copy
import pybullet as p
from ompl import util as ou
from ompl import base as ob
from ompl import geometric as og
from itertools import product
from .utils import *


INTERPOLATE_NUM = 500
DEFAULT_PLANNING_TIME = 5.0


# class PbOMPLRobot:
#     """
#     To use with Pb_OMPL. You need to construct a instance of this class and pass to PbOMPL.

#     Note:
#     This parent class by default assumes that all joints are acutated and should be planned. If this is not your desired
#     behaviour, please write your own inheritated class that overrides respective functionalities.
#     """

#     def __init__(self, id, joint_idx) -> None:
#         # Public attributes
#         self.id = id

#         # # prune fixed joints
#         # all_joint_num = p.getNumJoints(id)
#         # all_joint_idx = list(range(all_joint_num))
#         # joint_idx = [j for j in all_joint_idx if self._is_not_fixed(j)]

#         # TODO CHECK
#         # if with grippers, joint_idx should only include arm joints
        
#         self.num_dim = len(joint_idx)
#         self.joint_idx = joint_idx
#         print(self.joint_idx)
#         self.joint_bounds = []

#         # self.reset()

#     def _is_not_fixed(self, joint_idx):
#         joint_info = p.getJointInfo(self.id, joint_idx)
#         return joint_info[2] != p.JOINT_FIXED

#     def get_cur_state(self):
#         return copy.deepcopy(self.state)

#     def set_state(self, state):
#         """
#         Set robot state.
#         To faciliate collision checking
#         Args:
#             state: list[Float], joint values of robot
#         """
#         self._set_joint_positions(self.joint_idx, state)
#         self.state = state

#     def reset(self):
#         """
#         Reset robot state
#         Args:
#             state: list[Float], joint values of robot
#         """
#         state = [0] * self.num_dim
#         self._set_joint_positions(self.joint_idx, state)
#         self.state = state

#     def _set_joint_positions(self, joints, positions):
#         for joint, value in zip(joints, positions):
#             p.resetJointState(self.id, joint, value, targetVelocity=0)

class pb_ompl:
    def __init__(self, robot, obstacles=[]) -> None:
        """
        Args
            robot: A PbOMPLRobot instance.
            obstacles: list of obstacle ids. Optional.
        """
        self.robot = robot
        self.robot_id = robot.get_arm_id()
        self.obstacles = obstacles
        
        self.space = ob.RealVectorStateSpace(robot.get_DOF())

        bounds = ob.RealVectorBounds(robot.get_DOF())
        joint_bounds = self.robot.get_joint_bounds()
        for i, bound in enumerate(joint_bounds):
            bounds.setLow(i, bound[0])
            bounds.setHigh(i, bound[1])
            # print('-' * 20 + '\n' + '!debug:{}'.format(bound))
        self.space.setBounds(bounds)

        self.ss = og.SimpleSetup(self.space)
        self.ss.setStateValidityChecker(ob.StateValidityCheckerFn(self.is_state_valid))
        self.si = self.ss.getSpaceInformation()
        # self.si.setStateValidityCheckingResolution(0.005)
        # self.collision_fn = pb_utils.get_collision_fn(self.robot_id, self.robot.joint_idx, self.obstacles, [], True, set(),
        #                                                 custom_limits={}, max_distance=0, allow_collision_links=[])

        self.set_obstacles(obstacles)
        self.set_planner("RRTConnect")  # RRT by default

    def set_obstacles(self, obstacles):
        self.obstacles = obstacles

        # update collision detection
        self.setup_collision_detection(self.robot, self.obstacles)

    def add_obstacles(self, obstacle_id):
        self.obstacles.append(obstacle_id)

    def remove_obstacles(self, obstacle_id):
        self.obstacles.remove(obstacle_id)

    def is_state_valid(self, state):
        # satisfy bounds TODO
        # Should be unecessary if joint bounds is properly set

        # check self-collision
        # self.robot.set_state(self.state_to_list(state))
        self.robot.set_arm_to_joint_angles(self.state_to_list(state))
        for link1, link2 in self.check_link_pairs:
            if pairwise_link_collision(
                self.robot_id, link1, self.robot_id, link2
            ):
                # print(get_body_name(body), get_link_name(body, link1), get_link_name(body, link2))
                return False

        # check collision against environment
        for body1, body2 in self.check_body_pairs:
            if pairwise_collision(body1, body2):
                # print('body collision', body1, body2)
                # print(get_body_name(body1), get_body_name(body2))
                return False
        return True

    def setup_collision_detection(
        self, robot, obstacles, self_collisions=True, allow_collision_links=[]
    ):
        self.check_link_pairs = (
            get_self_link_pairs(self.robot_id, robot.get_joint_idx())
            if self_collisions
            else []
        )
        moving_links = frozenset(
            [
                item
                for item in get_moving_links(self.robot_id, robot.get_joint_idx())
                if not item in allow_collision_links
            ]
        )
        moving_bodies = [(self.robot_id, moving_links)]
        self.check_body_pairs = list(product(moving_bodies, obstacles))

    def set_planner(self, planner_name):
        """
        Note: Add your planner here!!
        """
        if planner_name == "PRM":
            self.planner = og.PRM(self.ss.getSpaceInformation())
        elif planner_name == "RRT":
            self.planner = og.RRT(self.ss.getSpaceInformation())
        elif planner_name == "RRTConnect":
            self.planner = og.RRTConnect(self.ss.getSpaceInformation())
        elif planner_name == "RRTstar":
            self.planner = og.RRTstar(self.ss.getSpaceInformation())
        elif planner_name == "EST":
            self.planner = og.EST(self.ss.getSpaceInformation())
        elif planner_name == "FMT":
            self.planner = og.FMT(self.ss.getSpaceInformation())
        elif planner_name == "BITstar":
            self.planner = og.BITstar(self.ss.getSpaceInformation())
        else:
            print("{} not recognized, please add it first".format(planner_name))
            return

        self.ss.setPlanner(self.planner)

    def plan_start_goal(self, start, goal, allowed_time=DEFAULT_PLANNING_TIME):
        """
        plan a path to gaol from the given robot start state
        """
        print("start_planning")
        print(self.planner.params())

        orig_robot_state = self.robot.get_arm_joint_angles()

        # set the start and goal states;
        s = ob.State(self.space)
        g = ob.State(self.space)
        for i in range(len(start)):
            s[i] = start[i]
            g[i] = goal[i]

        self.ss.setStartAndGoalStates(s, g)

        # attempt to solve the problem within allowed planning time
        solved = self.ss.solve(allowed_time)
        res = False
        sol_path_list = []
        if solved:
            print(
                "Found solution: interpolating into {} segments".format(INTERPOLATE_NUM)
            )
            # print the path to screen
            sol_path_geometric = self.ss.getSolutionPath()
            sol_path_geometric.interpolate(INTERPOLATE_NUM)
            sol_path_states = sol_path_geometric.getStates()
            sol_path_list = [self.state_to_list(state) for state in sol_path_states]
            # print(len(sol_path_list))
            # print(sol_path_list)
            for sol_path in sol_path_list:
                self.is_state_valid(sol_path)
            res = True
        else:
            print("No solution found")

        # reset robot state
        # self.robot.set_state(orig_robot_state)
        self.robot.set_arm_to_joint_angles(orig_robot_state)
        return res, sol_path_list

    def plan(self, goal, allowed_time=DEFAULT_PLANNING_TIME):
        """
        plan a path to gaol from current robot state
        """
        # start = self.robot.get_cur_state()
        start = self.robot.get_arm_joint_angles()
        return self.plan_start_goal(start, goal, allowed_time=allowed_time)

    def execute(self, path, dynamics=False):
        """
        Execute a planned plan. Will visualize in pybullet.
        Args:
            path: list[state], a list of state
            dynamics: allow dynamic simulation. If dynamics is false, this API will use robot.set_state(),
                      meaning that the simulator will simply reset robot's state WITHOUT any dynamics simulation. Since the
                      path is collision free, this is somewhat acceptable.
        """
        for q in path:
            if dynamics:
                for i in range(self.robot.get_DOF()):
                    p.setJointMotorControl2(
                        self.robot.id, i, p.POSITION_CONTROL, q[i], force=5 * 240.0
                    )
            else:
                # self.robot.set_state(q)
                self.robot.set_arm_to_joint_angles(q)
            p.stepSimulation()


    # -------------
    # Util
    # ------------

    def state_to_list(self, state):
        return [state[i] for i in range(self.robot.get_DOF())]