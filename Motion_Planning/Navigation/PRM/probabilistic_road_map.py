"""

Probabilistic Road Map (PRM) Planner

author: Atsushi Sakai (@Atsushi_twi)

"""

import math
import numpy as np
import matplotlib.pyplot as plt
from rtree import index
from scipy.spatial import KDTree
from Motion_Planning.Navigation.utils import *
from Motion_Planning.Robot import Pose

# parameter
N_SAMPLE = 2000  # number of sample_points
N_KNN = 10  # number of edge from one sampled point
MAX_EDGE_LEN = 30.0  # [m] Maximum edge length


class PRMPlanner:
    """
    Class for PRM planning
    """
    def __init__(
        self, 
        robot_size, 
        obstacles_bounds, 
        enable_plot = True
    ):
        self.robot_size = robot_size
        self.obstacles_bounds = obstacles_bounds
        self.robot_radius = robot_size / 2
        self.idx = index.Index()
        for id, obstacle_bounds in enumerate(self.obstacles_bounds):
            self.idx.insert(id, obstacle_bounds)
        self.enable_plot = enable_plot

    class Node:
        """
        Node class for dijkstra search
        """

        def __init__(self, x, y, cost, parent_index):
            self.x = x
            self.y = y
            self.cost = cost
            self.parent_index = parent_index

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," +\
                str(self.cost) + "," + str(self.parent_index)


    def plan(self, start_pose, goal_pose):
        """
        Find a path from a specified initial position to a goal position
        """
        start_position = start_pose.position[0:2]
        goal_position = goal_pose.position[0:2]
        self.start_position = start_position
        self.goal_position = goal_position
        self.area = AreaBounds(self.start_position,self.goal_position,self.obstacles_bounds)
        sample_x, sample_y = self.sample_points(start_pose.position[0], start_pose.position[1], goal_position[0], goal_position[1])
        road_map = self.generate_road_map(sample_x, sample_y)
        self.rx, self.ry = self.dijkstra_planning(start_pose.position[0], start_pose.position[1], goal_position[0], goal_position[1], road_map, sample_x, sample_y)
        
        # Draw final path
        if self.enable_plot:
            self.visual()

        self.path = [(x, y) for x, y in zip(self.rx, self.ry)]
        self.path.reverse()
        return self.path
    
    def visual(self):
        """
        Visualization of routes generated by TODO navigation algorithm
        """
        for (x_min, y_min, x_max, y_max) in self.obstacles_bounds:
            plot_rectangle(x_min, y_min, x_max, y_max)
        plt.plot(self.start_position[0], self.start_position[1], "^r")
        plt.plot(self.goal_position[0], self.goal_position[1], "^c")
        plt.plot(self.rx, self.ry, "-r")
        plt.pause(0.001)
        plt.show()
        plt.grid(True)
        plt.axis("equal")


    def is_collision(self, sx, sy, gx, gy):
        x = sx
        y = sy
        dx = gx - sx
        dy = gy - sy
        yaw = math.atan2(gy - sy, gx - sx)
        d = math.hypot(dx, dy)

        if d >= MAX_EDGE_LEN:
            return True

        D = self.robot_size
        n_step = round(d / D)

        for i in range(n_step):
            query_area = [x-self.robot_radius,y-self.robot_radius,x+self.robot_radius,y+self.robot_radius]
            intersected_ids = list(self.idx.intersection(query_area))
            if len(intersected_ids)>0:
                return True # collision
            x += D * math.cos(yaw)
            y += D * math.sin(yaw)

        # goal point check
        query_area = [gx-self.robot_radius,gy-self.robot_radius,gx+self.robot_radius,gy+self.robot_radius]
        intersected_ids = list(self.idx.intersection(query_area))
        if len(intersected_ids)>0:
            return True # collision

        return False  # OK


    def generate_road_map(self, sample_x, sample_y):
        """
        Road map generation

        sample_x: [m] x positions of sampled points
        sample_y: [m] y positions of sampled points
        robot_radius: Robot Radius[m]
        """

        road_map = []
        n_sample = len(sample_x)
        sample_kd_tree = KDTree(np.vstack((sample_x, sample_y)).T)

        for (i, ix, iy) in zip(range(n_sample), sample_x, sample_y):

            dists, indexes = sample_kd_tree.query([ix, iy], k=n_sample)
            edge_id = []

            for ii in range(1, len(indexes)):
                nx = sample_x[indexes[ii]]
                ny = sample_y[indexes[ii]]

                if not self.is_collision(ix, iy, nx, ny):
                    edge_id.append(indexes[ii])

                if len(edge_id) >= N_KNN:
                    break

            road_map.append(edge_id)

        return road_map


    def dijkstra_planning(self, sx, sy, gx, gy, road_map, sample_x, sample_y):
        """
        s_x: start x position [m]
        s_y: start y position [m]
        goal_x: goal x position [m]
        goal_y: goal y position [m]
        obstacle_x_list: x position list of Obstacles [m]
        obstacle_y_list: y position list of Obstacles [m]
        robot_radius: robot radius [m]
        road_map: ??? [m]
        sample_x: ??? [m]
        sample_y: ??? [m]

        @return: Two lists of path coordinates ([x1, x2, ...], [y1, y2, ...]), empty list when no path was found
        """

        start_node = self.Node(sx, sy, 0.0, -1)
        goal_node = self.Node(gx, gy, 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[len(road_map) - 2] = start_node

        path_found = True

        while True:
            if not open_set:
                print("Cannot find path")
                path_found = False
                break

            c_id = min(open_set, key=lambda o: open_set[o].cost)
            current = open_set[c_id]

            if c_id == (len(road_map) - 1):
                print("goal is found!")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            # Remove the item from the open set
            del open_set[c_id]
            # Add it to the closed set
            closed_set[c_id] = current

            # expand search grid based on motion model
            for i in range(len(road_map[c_id])):
                n_id = road_map[c_id][i]
                dx = sample_x[n_id] - current.x
                dy = sample_y[n_id] - current.y
                d = math.hypot(dx, dy)
                node = self.Node(sample_x[n_id], sample_y[n_id],
                            current.cost + d, c_id)

                if n_id in closed_set:
                    continue
                # Otherwise if it is already in the open set
                if n_id in open_set:
                    if open_set[n_id].cost > node.cost:
                        open_set[n_id].cost = node.cost
                        open_set[n_id].parent_index = c_id
                else:
                    open_set[n_id] = node

        if path_found is False:
            return [], []

        # generate final course
        rx, ry = [goal_node.x], [goal_node.y]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(n.x)
            ry.append(n.y)
            parent_index = n.parent_index

        return rx, ry


    def sample_points(self, sx, sy, gx, gy):
        max_x = self.area.x_max
        max_y = self.area.y_max
        min_x = self.area.x_min
        min_y = self.area.y_min

        sample_x, sample_y = [], []

        rng = np.random.default_rng()

        while len(sample_x) <= N_SAMPLE:
            tx = (rng.random() * (max_x - min_x)) + min_x
            ty = (rng.random() * (max_y - min_y)) + min_y

            query_area = [tx-self.robot_radius, ty-self.robot_radius, tx+self.robot_radius, ty+self.robot_radius]
            intersected_ids = list(self.idx.intersection(query_area))
            if len(intersected_ids)<=0:
                sample_x.append(tx)
                sample_y.append(ty)

        sample_x.append(sx)
        sample_y.append(sy)
        sample_x.append(gx)
        sample_y.append(gy)

        return sample_x, sample_y


def main():
    print(__file__ + " start!!")

    robot_size = 5.0  # [m]
    obstacles_bounds = [[-2.5,-2.5,2.5,2.5],[-2.5,2.5,2.5,62.5],[2.5,57.5,62.5,62.5],[57.5,-2.5,62.5,62.5],[-2.5,-2.5,62.5,2.5],[17.5,-2.5,22.5,40],[37.5,20,42.5,62.5]]
    
    # Set Initial parameters
    prm = PRMPlanner(
        robot_size=robot_size,
        obstacles_bounds = obstacles_bounds,
        enable_plot=True
    )

    # route plan
    start_pose = Pose([10.0, 10.0, 0.0], [0.0, 0.0, 0.0])
    goal_pose = Pose([50.0, 50.0, 0.0], [0.0, 0.0, 0.0])
    path = prm.plan(start_pose,goal_pose)

if __name__ == '__main__':
    main()
