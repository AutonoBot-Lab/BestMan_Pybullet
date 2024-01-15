# BestMan ⚙️ A Pybullet-based Mobile Manipulator Simulator

Welcome to the official repository for the BestMan Robot Simulator, integrated with Pybullet. This project provides a comprehensive simulation environment for the BestMan robot, a sophisticated machine featuring a robust base and a versatile arm (Ur5e).


## 💻 Installation

`git clone https://github.com/yding25/BestMan_Pybullet.git`

`cd BestMan_Pybullet`

`git submodule update --init --recursive`

## :mag_right: Project Structure
```
├── APIs_in_utils.txt
├── examples
│   ├── navigation_basic.py
│   ├── ...
├── Kitchen_models
├── tool
│   ├── capture_screen_front.py
│   ├── ...
├── URDF_models
├── URDF_robot
│   ├── segbot.urdf
│   ├── ur5e.urdf
│   └── ...
└── utils
    ├── pb_ompl.py
    ├── utils_Bestman.py
    ├── ....
```

## 👨‍💻 Basic Demos

**Navigation**

`python3 ./examples/navigation_basic.py`

<a href="https://www.youtube.com/watch?v=_tVbxgiM-5Q">
    <img src="https://img.youtube.com/vi/_tVbxgiM-5Q/0.jpg" alt="navigation" width="300" height="200">
</a>


**Manipulation**

`python3 ./examples/grasp_bowl_in_kitchen_v0.py`

<a href="https://www.youtube.com/watch?v=XnmEqOgxNM4">
    <img src="https://img.youtube.com/vi/XnmEqOgxNM4/0.jpg" alt="manipulation" width="300" height="200">
</a>

**Load Kitchen**

`python3 ./examples/load_kitchen_v0.py`

<a href="https://www.youtube.com/watch?v=hes7J-uy2DU">
    <img src="https://img.youtube.com/vi/hes7J-uy2DU/0.jpg" alt="kitchen" width="300" height="200">
</a>

**Manipulation + OMPL**

`python3 ./examples/grasp_bowl_from_drawer_in_kitchen0.py`

Demonstration Video

<a href="https://www.youtube.com/watch?v=f25d4N_Lv9w">
    <img src="https://img.youtube.com/vi/f25d4N_Lv9w/0.jpg" alt="OMPL" width="300" height="200">
</a>

###  :blue_book: Additional Resources

- **APIs_in_utils.txt**: A detailed list of common functions used in the utility scripts

