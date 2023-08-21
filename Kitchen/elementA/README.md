cd urdf/
rosrun xacro xacro --inorder kitchen_part_right.urdf.xacro > kitchen_part_right.urdf

roslaunch kitchen_description kitchen_rviz.launch gui:=True
(set fixed_frame to 'world')

Or easier:
roslaunch urdf_tutorial display.launch model:=urdf/kitchen_part_right.urdf.xacro
