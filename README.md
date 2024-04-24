![](docs/BestMan_logo.png)

<!-- # BestMan - A Pybullet-based Mobile Manipulator Simulator -->

[![GitHub license](https://img.shields.io/badge/license-MIT-blue.svg)](https://github.com/facebookresearch/home-robot/blob/main/LICENSE)
[![Python 3.8](https://img.shields.io/badge/python-3.8-blue.svg)](https://www.python.org/downloads/release/python-370/)

Welcome to the official repository of BestMan, a mobile manipulator simulator (with a wheel-base and arm) built on PyBullet.



## 💻 Installation

- Pull the repository and update the submodule

```
git clone https://github.com/AutonoBot-Lab/BestMan_Pybullet.git -b refactor
cd BestMan_Pybullet
git submodule init
git submodule update
```



### :shamrock: Anaconda

First install `Anaconda` or `minconda` on linux system and then perform the following steps：

```
cd Install

# Create conda environment from environment.yaml
conda env create -f environment.yaml
conda activate BestMan

# Install ompl
pip install ompl-1.6.0-cp38-cp38-manylinux_2_28_x86_64.whl

# run sh
bash conda_install.sh
```



### :shamrock: Docker

##### Windows

- Pull docker image from tencentyun

```
docker pull ccr.ccs.tencentyun.com/4090/bestman:v1
```

- Create docker container

```
docker run -it --gpus all --name BestMan ccr.ccs.tencentyun.com/4090/bestman:v1
```

- Install [VcXsrv Windows X Server](https://sourceforge.net/projects/vcxsrv/), Start and keep running in the background.

- Execute `echo $DISPLAY` inside the container, Make sure the result is `host.docker.internal:0` so that it can be visualized on the host machine, if not:

```
export DISPLAY=host.docker.internal:0
```



##### Linux





## 🔎 Project Structure

```
|-- Asset
|   |-- Generate_data
|   |-- Kitchen_models
|   |   ...
|   |-- Scene
|   |   |-- Kitchen.json
|   |   |-- Kitchen_bak.json
|   |   `-- test.py
|   |-- URDF_models
|   |   |-- README.md
|   |   |-- black_marker
|   |      ` ...   
|   |   ...
|   `-- mobile_manipulator
|       |-- arm
|       |   |-- panda
|       |   `-- ur5e
|       |-- base
|       |   `-- segbot
|       `-- end_effector
|           |-- gripper
|           |   `-- robotiq_2f85.urdf
|           `-- sucker
|               |-- long_vacuum_gripper
|               `-- vacuum_gripper
|-- CV
|-- Config
|   |-- default.yaml
|   |-- navigation_basic.yaml
|   `-- ...
|-- Controller
|   |-- PIDController.py
|-- Env
|   |-- Client.py
|   |-- Env.py
|   |-- Kitchen_v0.py
|   |-- Kitchen_v1.py
|   |-- Kitchen_v2.py
|   `-- __init__.py
|-- Examples
|   |-- navigation_basic.py
|   |-- ...
|-- Install
|   |-- conda_install.sh
|   |-- environment.yaml
|   |-- install-ompl-ubuntu.sh
|   `-- ompl-1.6.0-cp38-cp38-manylinux_2_28_x86_64.whl
|-- Motion_Planning
|   |-- Manipulation
|   |   |-- Collision
|   |   |   |-- Collision.py
|   |   |   |-- __init__.py
|   |   |   `-- utils.py
|   |   |-- OMPL_Planner.py
|   |   |-- __init__.py
|   |   |-- anygrasp_sdk
|   |   |   | ...
|   |-- Navigation
|   |   |-- A_star
|   |   |   |-- A_star.py
|   |   |-- PRM
|   |   |   |-- probabilistic_road_map.py
|   |   |-- RRT
|   |   |   |-- rrt.py
|   |   |-- TODO.md
|   |   |-- __init__.py
|   |   `-- utils.py
|   `-- Robot
|   |   |-- Bestman.py
|   |   |-- Pose.py
|   |   `-- __init__.py
|-- README.md
|-- SLAM
|-- Task_Planning
|-- Test
|   `-- test.py
|-- User_Interface
|-- Utils
|   |-- __init__.py
|   `-- load_config.py
|-- Visualization
    |-- Visualizer.py
    `-- __init__.py
`-- APIs_in_utils.txt
```



## 👨‍💻 Basic Demos

:shamrock: **Load Kitchens**

```
cd Examples
python3 load_kitchen.py
```

<table>
  <tr>
    <td>
      <a href="https://www.youtube.com/watch?v=hes7J-uy2DU">
        <img src="https://img.youtube.com/vi/hes7J-uy2DU/0.jpg" alt="kitchen"     width="250" height="200">
      </a>
    </td>
  </tr>
</table>


:shamrock: **Navigation**

```
cd Examples
python navigation_basic.py
```

<table>
  <tr>
    <td>
      <a href="https://www.youtube.com/watch?v=_tVbxgiM-5Q">
          <img src="https://img.youtube.com/vi/_tVbxgiM-5Q/0.jpg" alt="navigation" width="250" height="200">
      </a>
    </td>
  </tr>
</table>


:shamrock: **Manipulation**

```
cd Examples
python grasp_bowl_in_kitchen.py
```

```
cd Examples
python grasp_bowl_from_drawer_in_kitchen.py
```

<table>
  <tr>
    <td>
      <a href="https://www.youtube.com/watch?v=XnmEqOgxNM4">
        <img src="https://img.youtube.com/vi/XnmEqOgxNM4/0.jpg" alt="manipulation" width="250" height="200">
      </a>
    </td>
    <td>
      <a href="https://www.youtube.com/watch?v=f25d4N_Lv9w">
        <img src="https://img.youtube.com/vi/f25d4N_Lv9w/0.jpg" alt="Video 1" width="250" height="200">
      </a>
    </td>
    <td>
      <a href="https://www.youtube.com/watch?v=7gbh2OGFkCk">
        <img src="https://img.youtube.com/vi/7gbh2OGFkCk/0.jpg" alt="Video 2" width="250" height="200">
      </a>
    </td>
  </tr>
</table>



<!-- <a href="https://www.youtube.com/watch?v=f25d4N_Lv9w">
    <img src="https://img.youtube.com/vi/f25d4N_Lv9w/0.jpg" alt="OMPL" width="300" height="200">
</a>

<a href="https://www.youtube.com/watch?v=7gbh2OGFkCk">
    <img src="https://img.youtube.com/vi/7gbh2OGFkCk/0.jpg" alt="OMPL" width="300" height="200">
</a> -->



##  📘 Documents

:shamrock: **APIs_in_utils.txt**: A detailed list of common functions used in the utility scripts



##  🚀 Citation and Reference

If you find this work useful, please consider citing:

```
@article{ding2023task,
  title={Task and motion planning with large language models for object rearrangement},
  author={Ding, Yan and Zhang, Xiaohan and Paxton, Chris and Zhang, Shiqi},
  journal={arXiv preprint arXiv:2303.06247},
  year={2023}
}
@article{ding2023integrating,
  title={Integrating Action Knowledge and LLMs for Task Planning and Situation Handling in Open Worlds},
  author={Ding, Yan and Zhang, Xiaohan and Amiri, Saeid and Cao, Nieqing and Yang, Hao and Kaminski, Andy and Esselink, Chad and Zhang, Shiqi},
  journal={arXiv preprint arXiv:2305.17590},
  year={2023}
}
```
