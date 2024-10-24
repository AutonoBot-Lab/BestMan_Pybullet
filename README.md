<br>
<p align="center">
<h1 align="center"><strong>BestMan: A Modular Mobile Manipulator Platform for Embodied AI with Unified Simulation-Hardware APIs</strong></h1>
  <p align="center">
    Chongqing University&emsp;&emsp;&emsp;&emsp;Shanghai AI Laboratory&emsp;&emsp;&emsp;&emsp;Xi'an Jiaotong-Liverpool University
  </p>
</p>

<div id="top" align="center">

![](docs/_static/BestMan/BestMan_logo_AL.png)

<!-- # BestMan - A Pybullet-based Mobile Manipulator Simulator -->

[![arxiv](https://img.shields.io/badge/arxiv-2410.13407-orange)](http://arxiv.org/abs/2410.13407)
[![paper](https://img.shields.io/badge/Paper-%F0%9F%93%96-yellow)](https://arxiv.org/pdf/2410.13407)
[![GitHub license](https://img.shields.io/badge/license-MIT-blue.svg)](https://github.com/facebookresearch/home-robot/blob/main/LICENSE)
[![pre-commit](https://img.shields.io/badge/pre--commit-enabled-brightgreen?logo=pre-commit&logoColor=white)](https://github.com/pre-commit/pre-commit)
[![Code style: black](https://img.shields.io/badge/code%20style-black-000000.svg)](https://github.com/psf/black)
[![Imports: isort](https://img.shields.io/badge/%20imports-isort-%231674b1?style=flat)](https://timothycrosley.github.io/isort/)
[![Document](https://img.shields.io/badge/Document-%F0%9F%93%98-green)](https://bestman-pybullet.readthedocs.io)

![](docs/_static/other/picture.svg)

Welcome to the official repository of BestMan!

A mobile manipulator (with a wheel-base and arm) platform built on PyBullet simulation with unified hardware APIs.

</div>

## 📋 Contents

- [🔥 News](#-News)
- [🏠 Getting Started](#-Getting-Started)
- [👨‍💻 Basic Demos](#-Basic-Demos)
- [📝 TODO List](#-todo-list)
- [🤝 Reference](#-Reference)
- [👏 Acknowledgements](#-Acknowledgements)

## 🔥 News
- [2024-10] We release the [paper](http://arxiv.org/abs/2410.13407) of BestMan.

## 🏠 Getting Started

### Prerequisites
- Ubuntu 20.04, 22.04
- Conda
  
  - Python 3.8, 3.9, 3.10

### Installation

Pull the repository and update the submodule

```
git clone https://github.com/AutonoBot-Lab/BestMan_Pybullet.git
cd BestMan_Pybullet
git submodule update --init
```

### :shamrock: Conda

- Run the following script to add the project to the PYTHON search path
```
cd Install
chmod 777 pythonpath.sh
bash pythonpath.sh
source ~/.bashrc
```

- Install ffmpeg to enable video record
```
sudo apt update && sudo apt install ffmpeg
```

- Configure related libraries and links to support OpenGL rendering (If it already exists, skip this step.)
```
sudo apt update && sudo apt install -y libgl1-mesa-glx libglib2.0-0
sudo mkdir /usr/lib/dri
sudo ln -s /lib/x86_64-linux-gnu/dri/swrast_dri.so /usr/lib/dri/swrast_dri.so
```

- Install gcc/g++ 9 (If it already exists, skip this step.)
```
sudo apt install -y build-essential gcc-9 g++-9
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 9
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-9 9
sudo update-alternatives --config gcc  # choice gcc-9
sudo update-alternatives --config g++  # choice g++-9

# Make sure gcc and g++ versions are consistent (conda enviroment don't install gcc to prevent problems caused by inconsistent versions)
gcc -v
g++ -v
```

- Configure mamba to speed up the conda environment construction (Optional, skip if installation is slow or fails)
```
conda install mamba -n base -c conda-forge
```

- Create basic conda environment
```
conda(mamba) env create -f basic_env.yaml
conda(mamba) activate BestMan
```

- Install torch
```
conda(mamba) env update -f cuda116.yaml
```

- Install lang-segment-anything
```
pip install -U git+https://github.com/luca-medeiros/lang-segment-anything.git
```

- Install AnyGrasp

&emsp;&emsp;**Note**:
  
> &emsp;You need to get anygrasp [license and checkpoint](./Perception/Grasp_Pose_Estimation/AnyGrasp/README.md) to use it.

> &emsp;You need `export MAX_JOBS=2` in terminal; before pip install if you are running on an laptop due to [this issue](https://github.com/NVIDIA/MinkowskiEngine/issues/228).
```
# Install MinkowskiEngine
conda install pytorch=1.13.1 -c pytorch --force-reinstall
pip install -U git+https://github.com/NVIDIA/MinkowskiEngine -v --no-deps --global-option="--blas_include_dirs=${CONDA_PREFIX}/include" --global-option="--blas=openblas"

# Install graspnetAPI
pip install graspnetAPI

# Install pointnet2
cd third_party/pointnet2
python setup.py install

# Force reinstall to ensure version
pip install --force-reinstall opencv-python==4.1.2.30 numpy==1.23.5
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
- TBD
<br/>


## 👨‍💻 Basic Demos

### 🌏 Overview

<video src="https://github.com/user-attachments/assets/499aed7a-6756-4bf5-b25b-84ad1b23d6f9"></video>

### 🚀 Run

Enter `Examples` directory and run the demos. You can also modify the parameters corresponding to the demo.

### 🎇 Blender Render

`open microwave` demo in **Overview** before blender rendering:

<video src="https://github.com/user-attachments/assets/fb8ef3ea-d045-4bbf-a28f-0bec56930aae"></video>

<br/>

We have improved the [pybullet-blender-recorder](https://github.com/huy-ha/pybullet-blender-recorder) to import pybullet scene into blender for better rendering

If you want to enable **pybullet-blender-recorder**, please：

1. Install the `pyBulletSimImporter.py` plugin under **Visualization/blender-render** directory in blender (Edit->Preferences->Add-ons->Install) (test on **blender3.6.5**) , and enalbe this plugin.

<img width="1040" alt="image" src="https://github.com/user-attachments/assets/ab9e99c7-64c8-40fe-bbfe-edc0c786b812">
  
2. Set `blender: Ture` in **Config/***.yaml**.

3. After running the demo, a pkl file will be generated and saved in **Examples/record** dir

4. Import the pkl files into blender.

> Note: This will freeze the current blender window before the processing is completed, please wait.

<img width="1040" alt="image" src="https://github.com/user-attachments/assets/c0fe66e8-347e-4ecc-b367-8b0c3592d329">

<br/>
<br/>

> Note: If the demo contains too many frames, you can change `pyBulletSimImporter.py`: ANIM_OT_import_pybullet_sim(): **skip_frames** parameters and reinstall in blender to reduce the number of imported frames.
<br/>

## 📝 TODO List

- \[x\] Release the platform with basic modules、functions and demos.
- \[x\] Polish APIs, related codes, and release documentation.
- \[x\] Release the paper with framework and demos Introduction.
- \[ \] Release the baseline models and benchmark modules.
- \[ \] Dynamically integrate digital assets.
- \[ \] Comprehensive improvement and further updates.


## 🤝 Reference

If you find this work useful, please consider citing:

```
@inproceedings{Yang2024BestManAM,
  title={BestMan: A Modular Mobile Manipulator Platform for Embodied AI with Unified Simulation-Hardware APIs},
  author={Kui Yang and Nieqing Cao and Yan Ding and Chao Chen},
  year={2024},
  url={https://api.semanticscholar.org/CorpusID:273403368}
}
```

## 👏 Acknowledgements

We would like to express our sincere gratitude to all the individuals and organizations who contributed to this project.

For a detailed list of acknowledgements, please refer to [appendix](docs/appendix).
