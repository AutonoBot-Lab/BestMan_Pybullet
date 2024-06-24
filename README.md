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
git clone https://github.com/AutonoBot-Lab/BestMan_Pybullet.git --branch master
cd BestMan_Pybullet
git submodule init
git submodule update
```


### :shamrock: Conda

First install [Anaconda / minconda](https://docs.conda.io/projects/conda/en/latest/user-guide/install/linux.html) on linux system and then perform the following steps：

- Run the following script to add the project to the PYTHON search path
```
cd Install
sudo bash pythonpath.sh
source ~/.bashrc
```

- Install ffmpeg to enable video record
```
sudo apt update && sudo apt install ffmpeg
```

- Install shared file (If it already exists, skip this step.)
```
sudo apt install -y libgl1-mesa-glx libglib2.0-0
sudo mkdir /usr/lib/dri
sudo ln -s /lib/x86_64-linux-gnu/dri/swrast_dri.so /usr/lib/dri/swrast_dri.so
```

- Optional: Configure mamba to speed up the conda environment construction
```
# Install conda-libmamba-solver
conda install -n base conda-libmamba-solver
conda config --set solver libmamba

# Install mamba
conda install mamba -n base -c conda-forge
```

- Create conda environment (If mamba is configured, the command uses mamba, otherwise conda)
```
mamba(conda) env create -f basic_environment.yaml
mamba(conda) activate BestMan
```


### :shamrock: Docker

#### Windows

- Pull docker image from DockerHub

```
docker pull yk1127/bestman:v1
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


#### Linux
- TBD


## 🔎 Project Structure

```
cat Asset/project_structure.txt
```


## 👨‍💻 Basic Demos

```
cd Examples/
```

:shamrock: **Load Kitchens**


```
python load_kitchen.py
```

:shamrock: **Navigation**

```
python navigation_basic.py
```

<table>
  <tr>
    <td>
      <a href="https://www.youtube.com/watch?v=HW6oQhs_e5U">
          <img src="https://img.youtube.com/vi/HW6oQhs_e5U/0.jpg" alt="navigation" width="250" height="200">
      </a>
    </td>
    <td>
      <a href="https://www.youtube.com/watch?v=_tVbxgiM-5Q">
          <img src="https://img.youtube.com/vi/_tVbxgiM-5Q/0.jpg" alt="navigation" width="250" height="200">
      </a>
    </td>
  </tr>
</table>


:shamrock: **Manipulation**

```
python grasp_bowl_in_kitchen.py
```

```
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
