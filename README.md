## The code is a Pybullet simulator database for BestMan robot. The BestMan robot consists of a base and an arm (Ur5e).

<img src="./image/bestman.png" alt="bestman" width="300" height="300">

Step 0: Download the repo

`git clone https://github.com/yding25/BestMan_Pybullet.git`


Step 1: Download the object models

`cd BestMan_Pybullet`

`git submodule update --init --recursive`


Step 2: Configure a proper environment in Python 3

`pip install -e .`

Step 3: Run a demo (navigation)

`python ./examples/example_navigation.py`

<img src="image/example_navigation.gif" width="300" height="225">


Step 4: Run a demo (manipulation)

`python ./examples/example_manipulation.py`

<img src="image/example_manipulation.gif" width="300" height="225">


Step 5: Run a demo (kitchen)

`python ./examples/example_kitchen.py`

<img src="image/example_kitchen.gif" width="300" height="225">

Step 5: Run a demo (RL)

`python ./examples/example_RL.py`

<iframe width="560" height="315" src="https://www.youtube.com/embed/f25d4N_Lv9w?si=Dpn9K-QtkfcBdLuI" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

Note that "functions_in_utils.txt" lists common functions in "utils_X.py".

