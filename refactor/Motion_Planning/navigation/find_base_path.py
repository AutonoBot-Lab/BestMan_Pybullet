import numpy as np
import pybullet as p
import networkx as nwx
from Visualization.navigation_route_visual import navigation_route_visual

# use A* algorithm to find a Manhattan path
def find_base_path(
    bestman,
    init_base_position,
    goal_base_position,
    x_max=10,
    y_max=10,
    resolution=0.05,
    enable_accurate_occupancy_map=True,
    enable_plot=True,
):
    """
    Find a path from a specified initial position to a goal position in a 2D grid representation

    Args:
        init_base_position (Pose): The initial position of the robot.
        goal_base_position (Pose): The goal position of the robot.

    Returns:
        The function returns a list of waypoints that form a path from the initial position to the goal position.
        Each waypoint is a list [x, y] representing a position in the world coordinates.

    Note:
        A* for path searching in map: 10 meter * 10 meter
        resolution is 0.1 meter
        a 2D grid: 1 for obstacles
    """
    init_base_position = init_base_position.position[0:2]   # only care about x, y
    goal_base_position = goal_base_position.position[0:2]

    def to_grid_coordinates(point, resolution):
        return [
            int(round((coordinate + max_val) / resolution))
            for coordinate, max_val in zip(point, [x_max, y_max])
        ]

    def to_world_coordinates(point, resolution):
        return [
            (coordinate * resolution) - max_val
            for coordinate, max_val in zip(point, [x_max, y_max])
        ]

    # Defining the environment size (in meters) and resolution
    size_x = 2 * x_max
    size_y = 2 * y_max
    n_points_x = int(size_x / resolution)
    n_points_y = int(size_y / resolution)

    # Create a 2D grid representing the environment
    static_map = np.zeros((n_points_x, n_points_y))

    # get arm and base size in meters
    (
        min_x_base,
        min_y_base,
        _,
        max_x_base,
        max_y_base,
        _,
    ) = bestman.pb_client.get_bounding_box(bestman.base_id)
    
    (
        min_x_arm,
        min_y_arm,
        _,
        max_x_arm,
        max_y_arm,
        _,
    ) = bestman.pb_client.get_bounding_box(bestman.arm_id)
    
    robot_size = max(
        max_x_base - min_x_base,
        max_y_base - min_y_base,
        max_x_arm - min_x_arm,
        max_y_arm - min_y_arm,
    )
    # print("robot size:{}".format(robot_size))

    # Compute the number of grid cells to inflate around each obstacle
    inflate_cells = int(round(robot_size / 2 / resolution))

    if not enable_accurate_occupancy_map:
        # get occupancy map
        for obstacle_id in bestman.pb_client.obstacle_navigation_ids:
            aabb = bestman.pb_client.get_bounding_box(obstacle_id)
            for i in range(
                max(0, int((aabb[0] + x_max) / resolution) - inflate_cells),
                min(
                    n_points_x, int((aabb[3] + x_max) / resolution) + inflate_cells
                ),
            ):
                for j in range(
                    max(0, int((aabb[1] + y_max) / resolution) - inflate_cells),
                    min(
                        n_points_y,
                        int((aabb[4] + y_max) / resolution) + inflate_cells,
                    ),
                ):
                    static_map[i][j] = 1
    else:
        # get accurate occupancy map
        for obstacle_id in bestman.pb_client.obstacle_navigation_ids:
            link_ids = [
                i
                for i in range(
                    -1, p.getNumJoints(obstacle_id, physicsClientId=bestman.client_id)
                )
            ]
            for link_id in link_ids:
                aabb_link = p.getAABB(
                    obstacle_id, link_id, physicsClientId=bestman.client_id
                )
                aabb_link = list(aabb_link[0] + aabb_link[1])
                # print(
                #     "-" * 20
                #     + "\n"
                #     + "obstacle_id:{} link_id:{} aabb_link:{}".format(
                #         obstacle_id, link_id, aabb_link
                #     )
                # )
                
                for i in range(
                    max(
                        0, int((aabb_link[0] + x_max) / resolution) - inflate_cells
                    ),
                    min(
                        n_points_x,
                        int((aabb_link[3] + x_max) / resolution) + inflate_cells,
                    ),
                ):
                    for j in range(
                        max(
                            0,
                            int((aabb_link[1] + y_max) / resolution)
                            - inflate_cells,
                        ),
                        min(
                            n_points_y,
                            int((aabb_link[4] + y_max) / resolution)
                            + inflate_cells,
                        ),
                    ):
                        static_map[i][j] = 1

    init_base_position_grid = to_grid_coordinates(init_base_position, resolution)
    goal_base_position_grid = to_grid_coordinates(goal_base_position, resolution)

    # Make sure the positions are within the environment and not on the table
    assert (
        0 <= init_base_position_grid[0] < n_points_x
        and 0 <= init_base_position_grid[1] < n_points_y
    ), "Initial base position is out of boundary!"
    assert (
        0 <= goal_base_position_grid[0] < n_points_x
        and 0 <= goal_base_position_grid[1] < n_points_y
    ), "Goal base position is out of boundary!"
    assert (
        static_map[init_base_position_grid[0], init_base_position_grid[1]] != 1
    ), "Initial base position is in an obstacle!"
    assert (
        static_map[goal_base_position_grid[0], goal_base_position_grid[1]] != 1
    ), "Goal base position is in an obstacle!"

    # Convert the numpy grid map to NetworkX graph
    graph = nwx.grid_2d_graph(n_points_x, n_points_y)
    for i in range(n_points_x):
        for j in range(n_points_y):
            if static_map[i, j] == 1:
                graph.remove_node((i, j))

    # A* star algorithm
    path = nwx.astar_path(
        graph, tuple(init_base_position_grid), tuple(goal_base_position_grid)
    )

    # print("path:{}".format(path))

    if enable_plot:
        navigation_route_visual(static_map, path)

    # Convert grid coordinates back to world coordinates
    path = [to_world_coordinates(point, resolution) for point in path]
    # print('raw path:{}'.format(path))

    return path