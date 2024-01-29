# BestMan - A Pybullet-based Mobile Manipulator Simulator

Welcome to the official repository for the BestMan Robot Simulator, integrated with Pybullet. This project provides a comprehensive simulation environment for the BestMan robot, a sophisticated machine featuring a robust base and a versatile arm (Ur5e).


## 💻 Installation

`git clone --recurse-submodules https://github.com/yding25/BestMan_Pybullet.git`

`cd BestMan_Pybullet`

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

```
python3 ./examples/navigation_basic.py
```

<a href="https://www.youtube.com/watch?v=_tVbxgiM-5Q">
    <img src="https://img.youtube.com/vi/_tVbxgiM-5Q/0.jpg" alt="navigation" width="300" height="200">
</a>


**Manipulation**

```
python3 ./examples/grasp_bowl_in_kitchen_v0.py
```

<a href="https://www.youtube.com/watch?v=XnmEqOgxNM4">
    <img src="https://img.youtube.com/vi/XnmEqOgxNM4/0.jpg" alt="manipulation" width="300" height="200">
</a>

**Load Kitchens**

```
python3 ./examples/load_kitchen_v0.py
```

<a href="https://www.youtube.com/watch?v=hes7J-uy2DU">
    <img src="https://img.youtube.com/vi/hes7J-uy2DU/0.jpg" alt="kitchen" width="300" height="200">
</a>

**Manipulation + OMPL**

```
python3 ./examples/grasp_bowl_from_drawer_in_kitchen0.py
```

Demonstration Videos

<table>
  <tr>
    <td>
      <a href="https://www.youtube.com/watch?v=f25d4N_Lv9w">
        <img src="https://img.youtube.com/vi/f25d4N_Lv9w/0.jpg" alt="Video 1" width="300" height="200">
      </a>
    </td>
    <td>
      <a href="https://www.youtube.com/watch?v=7gbh2OGFkCk">
        <img src="https://img.youtube.com/vi/7gbh2OGFkCk/0.jpg" alt="Video 2" width="300" height="200">
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

###  :blue_book: Additional Resources

- **APIs_in_utils.txt**: A detailed list of common functions used in the utility scripts


###  🚀 Citation and Reference
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
@article{gao2023orla,
  title={ORLA*: Mobile Manipulator-Based Object Rearrangement with Lazy A},
  author={Gao, Kai and Ding, Yan and Zhang, Shiqi and Yu, Jingjin},
  journal={arXiv preprint arXiv:2309.13707},
  year={2023}
}
```