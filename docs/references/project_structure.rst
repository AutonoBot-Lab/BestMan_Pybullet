Project Structure Overview
==========================

.. code-block:: bash

    .
    ├── Asset
    │   ├── mobile_manipulator
    │   │   ├── arm
    │   │   ├── base
    │   │   └── end_effector
    │   └── Scene
    │       └── Kitchen.json
    ├── Config
    │   ├── blender_test.yaml
    │   ├── debug_set_arm.yaml
    │   ├── default.yaml
    │   ├── draw_AABB_fridge_door_link.yaml
    │   ├── grasp_bowl_from_drawer_in_kitchen.yaml
    │   ├── ...
    │   └── test_gripper.yaml
    ├── Controller
    │   └── PIDController.py
    ├── docs
    │   ├── ...
    │   └── requirements.txt
    ├── Env
    │   └── Client.py
    ├── Examples
    │   ├── image
    │   ├── log
    │   ├── record
    │   ├── blender_test.py
    │   ├── debug_set_arm.py
    │   ├── draw_fridge_door_link.py
    │   ├── ...
    │   └── open_microwave.py
    ├── Install
    │   ├── third_party
    │   │   ├── pointnet2
    │   │   └── ompl-1.6.0-cp38-cp38-manylinux_2_28_x86_64.whl
    │   ├── basic_environment.yaml
    │   ├── cuda113.yaml
    │   ├── cuda116.yaml
    │   └── pythonpath.sh
    ├── Motion_Planning
    │   ├── Manipulation
    │   │   ├── Collision
    │   │   └── OMPL_Planner
    │   └── Navigation
    │       ├── A_star
    │       ├── PRM
    │       ├── RRT
    │       ├── TODO.md
    │       └── utils.py
    ├── Perception
    │   ├── Find_object
    │   ├── Grasp_Pose_Estimation
    │   │   └── AnyGrasp
    │   └── Object_detection
    │       ├── Detic
    │       ├── Grounded_SAM
    │       └── Lang_SAM
    ├── RoboticsToolBox
    │   ├── Bestman_sim_panda.py
    │   ├── Bestman_sim.py
    │   ├── Bestman_sim_ur5e_vacuum_long.py
    │   └── Pose.py
    ├── SLAM
    │   ├── slam.py
    │   └── utils.py
    ├── User_Interface
    ├── Utils
    │   └── load_config.py
    ├── Visualization
    │   ├── blender_render
    │   │   ├── pyBulletSimImporter.py
    │   │   └── pyBulletSimRecorder.py
    │   ├── Camera.py
    │   ├── utils.py
    │   └── Visualizer.py
    ├── LICENSE.txt
    ├── README_bak.md
    └── README.md