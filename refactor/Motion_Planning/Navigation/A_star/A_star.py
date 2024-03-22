import numpy as np
import pybullet as p
import networkx as nwx
from Visualization.navigation_route_visual import navigation_route_visual


class AStarPlanner:
        
    def __init__(self, 
        robot_size, 
        obstacles_bounds, 
        resolution, 
        x_max = 10,
        y_max = 10,
        enable_plot=False
    ):
        self.robot_size = robot_size
        self.obstacles_bounds = obstacles_bounds
        self.resolution = resolution
        self.x_max = x_max
        self.y_max = y_max
        self.enable_plot = enable_plot
    
    # use A* algorithm to find a Manhattan path
    def plan(
        self, 
        start_pose,
        goal_pose
    ):
        """
        Find a path from a specified initial position to a goal position in a 2D grid representation

        Args:
            init_base_position (Pose): The initial position of the robot.
            goal_base_pose (Pose): The goal pose of the robot.

        Returns:
            The function returns a list of waypoints that form a path from the initial position to the goal position.
            Each waypoint is a list [x, y] representing a position in the world coordinates.

        Note:
            A* for path searching in map: 10 meter * 10 meter
            resolution is 0.1 meter
            a 2D grid: 1 for obstacles
        """
        
        # only care about x, y
        start_position = start_pose.position[0:2]
        goal_position = goal_pose.position[0:2]
        
        # grid to world coords
        def to_grid_coordinates(point, resolution):
            return [
                int(round((coordinate + max_val) / resolution))
                for coordinate, max_val in zip(point, [self.x_max, self.y_max])
            ]

        # world to grid coords
        def to_world_coordinates(point, resolution):
            return [
                (coordinate * resolution) - max_val
                for coordinate, max_val in zip(point, [self.x_max, self.y_max])
            ]

        # Defining the environment size (in meters) and resolution
        size_x = 2 * self.x_max
        size_y = 2 * self.y_max
        n_points_x = int(size_x / self.resolution)
        n_points_y = int(size_y / self.resolution)

        # Create a 2D grid representing the environment
        static_map = np.zeros((n_points_x, n_points_y))

        # Compute the number of grid cells to inflate around each obstacle
        inflate_cells = int(round(self.robot_size / 2 / self.resolution))

        # get occupancy map
        for obstacle_bound in self.obstacles_bounds:
            for i in range(
                max(0, int((obstacle_bound[0] + self.x_max) / self.resolution) - inflate_cells),
                min(
                    n_points_x, int((obstacle_bound[2] + self.x_max) / self.resolution) + inflate_cells
                ),
            ):
                for j in range(
                    max(0, int((obstacle_bound[1] + self.y_max) / self.resolution) - inflate_cells),
                    min(
                        n_points_y,
                        int((obstacle_bound[3] + self.y_max) / self.resolution) + inflate_cells,
                    ),
                ):
                    static_map[i][j] = 1

        start_grid = to_grid_coordinates(start_position, self.resolution)
        goal_grid = to_grid_coordinates(goal_position, self.resolution)

        # Make sure the positions are within the environment and not on the table
        assert (
            0 <= start_grid[0] < n_points_x
            and 0 <= start_grid[1] < n_points_y
        ), "Initial base position is out of boundary!"
        assert (
            0 <= goal_grid[0] < n_points_x
            and 0 <= goal_grid[1] < n_points_y
        ), "Goal base position is out of boundary!"
        assert (
            static_map[start_grid[0], start_grid[1]] != 1
        ), "Initial base position is in an obstacle!"
        assert (
            static_map[goal_grid[0], goal_grid[1]] != 1
        ), "Goal base position is in an obstacle!"

        # Convert the numpy grid map to NetworkX graph
        graph = nwx.grid_2d_graph(n_points_x, n_points_y)
        for i in range(n_points_x):
            for j in range(n_points_y):
                if static_map[i, j] == 1:
                    graph.remove_node((i, j))

        # A* star algorithm
        path = nwx.astar_path(
            graph, tuple(start_grid), tuple(goal_grid)
        )

        # print("path:{}".format(path))

        if self.enable_plot:
            navigation_route_visual(static_map, path)

        # Convert grid coordinates back to world coordinates
        path = [to_world_coordinates(point, self.resolution) for point in path]
        # print('raw path:{}'.format(path))

        return path