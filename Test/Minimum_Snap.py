import numpy as np
import cvxpy as cp
import matplotlib.pyplot as plt


def minimum_snap_trajectory_2d(waypoints, times):
    n = len(waypoints) - 1  # number of segments
    d = 2  # dimensions (x, y)

    # Variables for polynomial coefficients of each segment
    coeffs = [cp.Variable((8, d)) for _ in range(n)]

    # Objective function: Minimize snap (fourth derivative)
    objective = cp.Minimize(
        sum(
            cp.sum_squares(
                24 * coeffs[i][4]
                + 120 * coeffs[i][5] * (times[i + 1] - times[i])
                + 360 * coeffs[i][6] * (times[i + 1] - times[i]) ** 2
                + 840 * coeffs[i][7] * (times[i + 1] - times[i]) ** 3
            )
            for i in range(n)
        )
    )

    # Constraints
    constraints = []

    # Initial and final constraints
    for dim in range(d):
        constraints += [
            coeffs[0][0, dim] == waypoints[0, dim],
            coeffs[0][1, dim] == 0,
            coeffs[0][2, dim] == 0,
            coeffs[0][3, dim] == 0,
            coeffs[-1][0, dim] == waypoints[-1, dim],
            coeffs[-1][1, dim] == 0,
            coeffs[-1][2, dim] == 0,
            coeffs[-1][3, dim] == 0,
        ]

    # Continuity constraints at waypoints
    for i in range(n - 1):
        for dim in range(d):
            constraints += [
                coeffs[i][0, dim]
                + coeffs[i][1, dim] * (times[i + 1] - times[i])
                + coeffs[i][2, dim] * (times[i + 1] - times[i]) ** 2
                + coeffs[i][3, dim] * (times[i + 1] - times[i]) ** 3
                + coeffs[i][4, dim] * (times[i + 1] - times[i]) ** 4
                + coeffs[i][5, dim] * (times[i + 1] - times[i]) ** 5
                + coeffs[i][6, dim] * (times[i + 1] - times[i]) ** 6
                + coeffs[i][7, dim] * (times[i + 1] - times[i]) ** 7
                == coeffs[i + 1][0, dim],
                coeffs[i][1, dim]
                + 2 * coeffs[i][2, dim] * (times[i + 1] - times[i])
                + 3 * coeffs[i][3, dim] * (times[i + 1] - times[i]) ** 2
                + 4 * coeffs[i][4, dim] * (times[i + 1] - times[i]) ** 3
                + 5 * coeffs[i][5, dim] * (times[i + 1] - times[i]) ** 4
                + 6 * coeffs[i][6, dim] * (times[i + 1] - times[i]) ** 5
                + 7 * coeffs[i][7, dim] * (times[i + 1] - times[i]) ** 6
                == coeffs[i + 1][1, dim],
                2 * coeffs[i][2, dim]
                + 6 * coeffs[i][3, dim] * (times[i + 1] - times[i])
                + 12 * coeffs[i][4, dim] * (times[i + 1] - times[i]) ** 2
                + 20 * coeffs[i][5, dim] * (times[i + 1] - times[i]) ** 3
                + 30 * coeffs[i][6, dim] * (times[i + 1] - times[i]) ** 4
                + 42 * coeffs[i][7, dim] * (times[i + 1] - times[i]) ** 5
                == 2 * coeffs[i + 1][2, dim],
                6 * coeffs[i][3, dim]
                + 24 * coeffs[i][4, dim] * (times[i + 1] - times[i])
                + 60 * coeffs[i][5, dim] * (times[i + 1] - times[i]) ** 2
                + 120 * coeffs[i][6, dim] * (times[i + 1] - times[i]) ** 3
                + 210 * coeffs[i][7, dim] * (times[i + 1] - times[i]) ** 4
                == 6 * coeffs[i + 1][3, dim],
            ]

    # Waypoint constraints
    for i in range(1, n):
        for dim in range(d):
            constraints.append(coeffs[i][0, dim] == waypoints[i, dim])

    # Solve the optimization problem
    prob = cp.Problem(objective, constraints)
    prob.solve(verbose=True)  # 添加verbose=True以获取求解过程的详细信息

    if prob.status not in ["infeasible", "unbounded"]:
        # Generate trajectory points
        traj = []
        for i in range(n):
            t = np.linspace(times[i], times[i + 1], 100)
            segment = np.array(
                [
                    np.poly1d(coeffs[i].value[:, dim][::-1])(t - times[i])
                    for dim in range(d)
                ]
            ).T
            traj.append(segment)

        traj = np.concatenate(traj)
        return traj
    else:
        print("Optimization problem is infeasible or unbounded.")
        print(f"Problem status: {prob.status}")
        return None


# Define waypoints and times
waypoints = np.array([[0, 0], [2, 3], [5, 1], [8, 4], [10, 0]])
times = np.array([0, 1, 2, 3, 4])

# Get the optimized trajectory
trajectory = minimum_snap_trajectory_2d(waypoints, times)

if trajectory is not None:
    # Plot the trajectory
    plt.plot(trajectory[:, 0], trajectory[:, 1], label="Optimized Trajectory")
    plt.scatter(waypoints[:, 0], waypoints[:, 1], color="red", label="Waypoints")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.legend()
    plt.show()
else:
    print("No valid trajectory found.")
