## The code is a Pybullet simulator database for BestMan robot. The BestMan robot consists of a base and an arm (Ur5e).

<!-- <img src="./image/bestman.png" alt="bestman" width="300" height="300">
 -->

Step 0: Download the repo

`git clone https://github.com/yding25/BestMan_Pybullet.git`


Step 1: Download the object models

`cd BestMan_Pybullet`

`git submodule update --init --recursive`


Step 2: Run a demo (navigation)

`python ./examples/navigation_basic.py`

#### Youtube Video
[![navigation](https://img.youtube.com/vi/_tVbxgiM-5Q/0.jpg)](https://www.youtube.com/watch?v=_tVbxgiM-5Q)


Step 3: Run a demo (manipulation)

`python ./examples/grasp_bowl_in_kitchen_v0.py`

#### Youtube Video
[![navigation](https://img.youtube.com/vi/XnmEqOgxNM4/0.jpg)](https://www.youtube.com/watch?v=XnmEqOgxNM4)


Step 4: Run a demo (kitchen)

`python ./examples/load_kitchen_v0.py`

#### Youtube Video
[![navigation](https://img.youtube.com/vi/hes7J-uy2DU/0.jpg)](https://www.youtube.com/watch?v=hes7J-uy2DU)


Step 5: Run a demo (manipulation + OMPL)

`python ./examples/grasp_bowl_from_drawer_in_kitchen0.py`

#### Youtube Video
[![manipulation + OMPL](https://img.youtube.com/vi/f25d4N_Lv9w/0.jpg)](https://www.youtube.com/watch?v=f25d4N_Lv9w)


### Note that "functions_in_utils.txt" lists common functions in "utils_X.py".

