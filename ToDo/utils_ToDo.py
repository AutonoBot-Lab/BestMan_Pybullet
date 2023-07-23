"""
Trajectory planning
input: joint angles
"""


class RRT_joint:
    class Node:
        def __init__(self, q):
            self.q = q  # Node configuration
            self.parent = None  # Parent node

    def __init__(
        self,
        start,
        goal,
        obstacle_list,
        arm_id,
        base_id,
        expand_dis=0.02,
        path_resolution=0.02,
        goal_sample_rate=50,
        max_iter=500,
    ):
        self.start = self.Node(start)  # Starting node
        self.goal = self.Node(goal)  # Goal node
        self.expand_dis = expand_dis  # Distance for expansion
        self.path_resolution = path_resolution  # Resolution of the path
        self.goal_sample_rate = goal_sample_rate  # Rate of sampling the goal
        self.max_iter = max_iter  # Maximum iterations
        self.obstacle_list = obstacle_list  # List of obstacles
        self.node_list = []  # List of nodes
        self.q_range = [
            [-np.pi, np.pi],
            [-np.pi, np.pi],
            [-np.pi, np.pi],
            [-np.pi, np.pi],
            [-np.pi, np.pi],
            [-np.pi, np.pi],
        ]  # Range of random samples
        self.arm_id = arm_id  # Id of the robot in pybullet
        self.base_id = base_id  # Id of the robot in pybullet

    def plan(self):
        self.node_list = [self.start]  # Initialize node list with the start node
        for i in range(self.max_iter):
            rnd_node = self.get_random_node(i)  # Generate a random node
            nearest_node = self.get_nearest_node(
                self.node_list, rnd_node
            )  # Find the nearest existing node to the random node
            new_node = self.steer(
                nearest_node, rnd_node, self.expand_dis
            )  # Steer towards the random node from the nearest node

            self.set_robot_joints(
                new_node.q
            )  # Set robot joints to new node configuration

            if not self.check_collision(new_node):  # Check if there's a collision
                self.node_list.append(new_node)  # If no collision, append the new node

                # Check if we're close to the goal
                if self.calc_dist_to_goal(self.node_list[-1].q) <= self.expand_dis:
                    final_node = self.steer(
                        self.node_list[-1], self.goal, self.expand_dis
                    )  # Steer towards the goal
                    self.set_robot_joints(
                        final_node.q
                    )  # Set robot joints to final node configuration
                    # If no collision in final path, return the final path
                    if not self.check_collision(final_node):
                        return self.generate_final_course(len(self.node_list) - 1)
        return None  # If no path found, return None

    def steer(self, from_node, to_node, extend_length=float("inf")):
        new_node = self.Node(np.array(from_node.q))  # Create a new node
        direction = (to_node.q - from_node.q)
        norm = np.linalg.norm(direction)
        if norm == 0:
            print("Warning: attempting to normalize a zero vector")
        else:
            direction /= norm
        d = np.linalg.norm(from_node.q - to_node.q)  # Distance between nodes
        if extend_length > d:
            extend_length = d
        n_expand = math.floor(
            extend_length / self.path_resolution
        )  # Number of expansions
        for _ in range(int(n_expand)):
            new_node.q += self.path_resolution * direction  # Extend the path
        d = np.linalg.norm(new_node.q - to_node.q)
        if d <= self.path_resolution:
            new_node.q = to_node.q  # If close enough, just assign the node directly
        new_node.parent = from_node  # Assign parent
        return new_node  # Return the new node

    def generate_final_course(self, goal_ind):
        path = [self.goal.q]  # Start path with the goal node
        node = self.node_list[goal_ind]  # Get the node at the goal index
        # Iterate back through the parent nodes to generate the final path
        while node.parent is not None:
            path.append(node.q)
            node = node.parent
        path.append(node.q)
        return path  # Return the final path

    def calc_dist_to_goal(self, q):
        return np.linalg.norm(q - self.goal.q)  # Calculate distance to goal

    def get_random_node(self, iteration):
        rnd_q = []
        if iteration > 10:
            goal_sample_rate = self.goal_sample_rate + (iteration / self.max_iter) * (
                100 - self.goal_sample_rate
            )
        else:
            goal_sample_rate = self.goal_sample_rate

        if (
            random.randint(0, 100) > goal_sample_rate
        ):  # Randomly sample in the configuration space
            for min_q, max_q in self.q_range:
                rnd_q.append(random.uniform(min_q, max_q))
            rnd = self.Node(np.array(rnd_q))
        else:  # Sometimes sample directly at the goal
            rnd = self.Node(self.goal.q)
        return rnd  # Return the random node

    def get_nearest_node(self, node_list, rnd_node):
        dlist = [
            np.linalg.norm(node.q - rnd_node.q) for node in node_list
        ]  # Calculate distance from each node to the random node
        minind = dlist.index(min(dlist))  # Get the index of the nearest node
        return node_list[minind]  # Return the nearest node

    def check_collision(self, node):
        self.set_robot_joints(node.q)  # set robot joint to the given configuration

        # Iterate through all links of the robot
        # for link_index_base in range(p.getNumJoints(self.base_id)):
        for link_index_base in [2]:
            for link_index_arm in range(p.getNumJoints(self.arm_id)):
                distance = 0.1  # maximal collision distance
                # print('bodyA:{}'.format(self.arm_id))
                # print('bodyB:{}'.format(self.base_id))
                # print('link_index_base:{}'.format(link_index_base))
                # print('link_index_arm:{}'.format(link_index_arm))
                closest_points = p.getClosestPoints(
                    bodyA=self.arm_id,
                    bodyB=self.base_id,
                    linkIndexA=link_index_arm,
                    linkIndexB=link_index_base,
                    distance=distance
                )  # Get the list of closest points
                if (
                    len(closest_points) > 0.0 and closest_points[0][8] <= 0.01
                ):  # If the minimum distance is less than or equal to 0
                    return True  # If there's a collision return True

        return False  # If no collision, return False

    def set_robot_joints(self, joint_angles):
        for joint_index in range(6):
            p.resetJointState(
                bodyUniqueId=self.arm_id,
                jointIndex=joint_index,
                targetValue=joint_angles[joint_index],
            )
        p.stepSimulation()