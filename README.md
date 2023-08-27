## The code is a Pybullet simulator database for BestMan robot. The BestMan robot consists of a base and an arm (Ur5e).

<!-- <img src="./image/bestman.png" alt="bestman" width="300" height="300">
 -->

Step 0: Download the repo

`git clone https://github.com/yding25/BestMan_Pybullet.git`


Step 1: Download the object models

`cd BestMan_Pybullet`

`git submodule update --init --recursive`


Step 2: Configure a proper environment in Python 3

`pip install -e .`

Step 3: Run a demo (navigation)

`python ./examples/example_navigation.py`

#### Youtube Video
[![navigation](https://img.youtube.com/vi/_tVbxgiM-5Q/0.jpg)](https://www.youtube.com/watch?v=_tVbxgiM-5Q)



Step 4: Run a demo (manipulation)

`python ./examples/example_manipulation.py`

#### Youtube Video
[![navigation](https://img.youtube.com/vi/XnmEqOgxNM4/0.jpg)](https://www.youtube.com/watch?v=XnmEqOgxNM4)


Step 5: Run a demo (kitchen)

`python ./examples/example_kitchen.py`

#### Youtube Video
[![navigation](https://img.youtube.com/vi/hes7J-uy2DU/0.jpg)](https://www.youtube.com/watch?v=hes7J-uy2DU)

Step 5: Run a demo (RL)

`python ./examples/example_RL.py`

Step 6: Run a demo (manipulation + OMPL)

`python ./examples/example_draw_pickup.py`

#### Youtube Video
[![manipulation + OMPL](https://img.youtube.com/vi/f25d4N_Lv9w/0.jpg)](https://www.youtube.com/watch?v=f25d4N_Lv9w)


### Note that "functions_in_utils.txt" lists common functions in "utils_X.py".

