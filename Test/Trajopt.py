import numpy as np
import trajoptpy
import json
import matplotlib.pyplot as plt

def optimize_trajectory_2d(waypoints):
    n_points = len(waypoints)
    waypoints = np.array(waypoints)
    
    # Define the optimization problem
    request = {
        "basic_info": {
            "n_steps": n_points,
            "manip": "ur5",
            "start_fixed": True
        },
        "costs": [
            {
                "type": "joint_vel",
                "params": {"coeffs": [1]}
            },
            {
                "type": "collision",
                "params": {
                    "coeffs": [20],
                    "dist_pen": [0.1]
                }
            }
        ],
        "constraints": [
            {
                "type": "joint",
                "params": {"vals": waypoints[0].tolist()}
            }
        ] + [
            {
                "type": "joint",
                "params": {"vals": waypoints[i].tolist()}
            } for i in range(1, n_points - 1)
        ] + [
            {
                "type": "joint",
                "params": {"vals": waypoints[-1].tolist()}
            }
        ],
        "init_info": {
            "type": "straight_line",
            "endpoint": waypoints[-1].tolist()
        }
    }

    # Optimize the trajectory
    s = json.dumps(request)
    prob = trajoptpy.ConstructProblem(s, trajoptpy.GetEnv())
    result = trajoptpy.OptimizeProblem(prob)

    # Extract the optimized trajectory
    traj = np.array(result.GetTraj())
    return traj

# Define waypoints (2D points)
waypoints = [
    [0, 0],
    [2, 3],
    [5, 1],
    [8, 4],
    [10, 0]
]

# Optimize the trajectory
optimized_trajectory = optimize_trajectory_2d(waypoints)

# Plot the optimized trajectory
plt.plot(optimized_trajectory[:, 0], optimized_trajectory[:, 1], label='Optimized Trajectory')
plt.scatter(np.array(waypoints)[:, 0], np.array(waypoints)[:, 1], color='red', label='Waypoints')
plt.xlabel('X')
plt.ylabel('Y')
plt.legend()
plt.show()
