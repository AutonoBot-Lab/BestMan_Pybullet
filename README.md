![](docs/_static/BestMan_logo.png)

<!-- # BestMan - A Pybullet-based Mobile Manipulator Simulator -->

[![GitHub license](https://img.shields.io/badge/license-MIT-blue.svg)](https://github.com/facebookresearch/home-robot/blob/main/LICENSE)
[![Ubuntu 22.04](https://img.shields.io/badge/Ubuntu-22.04-orange.svg)](https://releases.ubuntu.com/22.04/)
[![Python 3.8](https://img.shields.io/badge/python-3.8-blue.svg)](https://www.python.org/downloads/release/python-370/)
[![pre-commit](https://img.shields.io/badge/pre--commit-enabled-brightgreen?logo=pre-commit&logoColor=white)](https://github.com/pre-commit/pre-commit)
[![Code style: black](https://img.shields.io/badge/code%20style-black-000000.svg)](https://github.com/psf/black)
[![Imports: isort](https://img.shields.io/badge/%20imports-isort-%231674b1?style=flat)](https://timothycrosley.github.io/isort/)

Welcome to the official repository of BestMan, a mobile manipulator simulator (with a wheel-base and arm) built on PyBullet.



## 💻 Installation

- Pull the repository and update the submodule

```
git clone https://github.com/AutonoBot-Lab/BestMan_Pybullet.git
cd BestMan_Pybullet
git submodule init
git submodule update
```

### :shamrock: Conda

First install `Anaconda` or `minconda` on linux system and then perform the following steps：

- Run the following script to add the project to the PYTHON search path
```
cd Install
chmod 777 pythonpath.sh
bash pythonpath.sh
source ~/.bashrc
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
conda(mamba) env create -f basic_environment.yaml
conda(mamba) activate BestMan

# Install torch
conda(mamba) env update -f cuda116.yaml

# Install lang-segment-anything
pip install -U git+https://github.com/luca-medeiros/lang-segment-anything.git

# Install MinkowskiEngine
pip install -U git+https://github.com/NVIDIA/MinkowskiEngine -v --no-deps --global-option="--blas_include_dirs=${CONDA_PREFIX}/include" --global-option="--blas=openblas"

# Install graspnetAPI
pip install graspnetAPI

# Install pointnet2
cd third_party/pointnet2
python setup.py install
```

- AnyGrasp License 
  
  You need to get anygrasp [license and checkpoint](./Perception/Grasp_Pose_Estimation/AnyGrasp/README.md) to use it.
<br/>


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


## 🔎 Project Structure & API References

- [Project Structure](docs/project_structure.txt)
- [API References](https://bestman-pybullet.readthedocs.io)
<br/>


## 👨‍💻 Basic Demos

&emsp;&emsp;We have supplemented and improved the [pybullet-blender-recorder](https://github.com/huy-ha/pybullet-blender-recorder) code base, importing the images in the pybullet scene into blender for rendering, which improves the rendering effect. For simple scenes and tasks, the import can be completed within 2 minutes, and for complex scenes and tasks, the import can be completed within half an hour.

<br/>
First, Enter directory Examples:

```
cd Examples
```

Below are some examples and their rendering in Blender


:shamrock: **Navigation**

```
python navigation_basic.py
```

<div style="display: flex; justify-content: space-between;">
  <div style="flex: 1; margin-right: 10px;">
    <!-- 使用 <img> 标签嵌入图片 -->
    <img src="https://github.com/user-attachments/assets/63fe074e-ba27-4de8-8095-99289552b17a" alt="Image Description" width="100%" />
  </div>
  <div style="flex: 1; margin-left: 10px;">
    <!-- 使用 <video> 标签嵌入视频 -->
    <video controls width="100%">
      <source src="https://github.com/user-attachments/assets/4e87e324-6621-4e47-810d-31c51582fb05" type="video/mp4">
      Your browser does not support the video tag.
    </video>
  </div>
</div>

<br/>


:shamrock: **Manipulation**

- Open Fridge

```
python open_fridge.py
```

https://github.com/user-attachments/assets/ffdb8468-5125-4d72-aa5b-4c237ac58a57 

https://github.com/user-attachments/assets/ed07b856-74ce-4299-9ba5-9de012b9eef5 
<br/>


- Open microwave

```
python open_microwave.py
```

https://github.com/user-attachments/assets/df0dbfea-a91a-43bd-8204-c61745aa40d9 

https://github.com/user-attachments/assets/77530f8d-30fb-471c-8e6d-40f8dddfd56a  
<br/>


- Grasp bowl on table use sucker

```
python grasp_bowl_on_table_sucker.py
```

https://github.com/user-attachments/assets/55e9f3ec-6c42-4f90-a5b1-b38d2c90e6b9 

https://github.com/user-attachments/assets/76909a22-e268-44b5-9415-4ef42f528f14 

<br/>


- Grasp lego on table use gripper

```
python grasp_lego_on_table_gripper.py
```

https://github.com/user-attachments/assets/2260d167-6b13-4281-99fe-a6fcf42cf3d8  

https://github.com/user-attachments/assets/beb7d70c-fa83-4229-a0dc-82eb95da92ff  

<br/>


- Move bowl from drawer to table

```
python move_bowl_from_drawer_to_table.py
```

https://github.com/user-attachments/assets/d48ea499-eac4-4810-88fe-306f71504216 

https://github.com/user-attachments/assets/580e658a-fd33-49d5-ad1c-ae513974a807 

<br/>


##  :handshake: Reference

If you find this work useful, please consider citing:

```
@inproceedings{ding2023task,
  title={Task and motion planning with large language models for object rearrangement},
  author={Ding, Yan and Zhang, Xiaohan and Paxton, Chris and Zhang, Shiqi},
  booktitle={2023 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages={2086--2092},
  year={2023},
  organization={IEEE}
}

@article{ding2023integrating,
  title={Integrating action knowledge and LLMs for task planning and situation handling in open worlds},
  author={Ding, Yan and Zhang, Xiaohan and Amiri, Saeid and Cao, Nieqing and Yang, Hao and Kaminski, Andy and Esselink, Chad and Zhang, Shiqi},
  journal={Autonomous Robots},
  volume={47},
  number={8},
  pages={981--997},
  year={2023},
  publisher={Springer}
}
```
