![](docs/BestMan_logo.png)

<!-- # BestMan - A Pybullet-based Mobile Manipulator Simulator -->

[![GitHub license](https://img.shields.io/badge/license-MIT-blue.svg)](https://github.com/facebookresearch/home-robot/blob/main/LICENSE)
[![Ubuntu 20.04](https://img.shields.io/badge/Ubuntu-20.04-orange.svg)](https://releases.ubuntu.com/20.04/)
[![Python 3.8](https://img.shields.io/badge/python-3.8-blue.svg)](https://www.python.org/downloads/release/python-370/)
[![pre-commit](https://img.shields.io/badge/pre--commit-enabled-brightgreen?logo=pre-commit&logoColor=white)](https://github.com/pre-commit/pre-commit)
[![Code style: black](https://img.shields.io/badge/code%20style-black-000000.svg)](https://github.com/psf/black)
[![Imports: isort](https://img.shields.io/badge/%20imports-isort-%231674b1?style=flat)](https://timothycrosley.github.io/isort/)

Welcome to the official repository of BestMan, a mobile manipulator simulator (with a wheel-base and arm) built on PyBullet.



## 💻 Installation

- Pull the repository and update the submodule

```
git clone https://github.com/AutonoBot-Lab/BestMan_Pybullet.git -b refactor
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

- Install shared file (If it already exists, skip this step.)
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

# Make sure gcc and g++ versions are consistent
gcc -v
g++ -v
```

- Configure mamba to speed up the conda environment construction
```
# Install conda-libmamba-solver
conda install -n base conda-libmamba-solver
conda config --set solver libmamba

# Install mamba
conda install mamba -n base -c conda-forge
```

- Create conda environment
```
mamba env create -f basic_environment.yaml
mamba activate BestMan

# Install torch
mamba env update -f cuda116.yaml

# Install MinkowskiEngine
pip install -U git+https://github.com/NVIDIA/MinkowskiEngine -v --no-deps --global-option="--blas_include_dirs=${CONDA_PREFIX}/include" --global-option="--blas=openblas"

# Install lang-segment-anything
pip install -U git+https://github.com/luca-medeiros/lang-segment-anything.git

# Install pointnet2
cd third_party/pointnet2
python setup.py install
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



## 🔎 Project Structure

```
cat Asset/project_structure.txt
```



## 👨‍💻 Basic Demos

​		We have supplemented and improved the [pybullet-blender-recorder](https://github.com/huy-ha/pybullet-blender-recorder) code base, importing the images in the pybullet scene into blender for rendering, which improves the rendering effect. For simple scenes and tasks, the import can be completed within 2 minutes, and for complex scenes and tasks, the import can be completed within half an hour.

​		Below are some examples and their rendering in Blender:

```
cd Examples
```



:shamrock: **Navigation**

```
python navigation_basic.py
```

https://github.com/user-attachments/assets/b62e8a39-ae3f-4e55-8035-e0592fdec6ac

https://github.com/user-attachments/assets/ade4f0cf-d120-402a-b7b7-0f645832f0d8



:shamrock: **Manipulation**

```
python open_fridge.py
```

https://github.com/user-attachments/assets/ffdb8468-5125-4d72-aa5b-4c237ac58a57



##  📘 Documents

:shamrock: **APIs_in_utils.txt**: A detailed list of common functions used in the utility scripts



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
