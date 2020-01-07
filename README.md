# RosPerception

0.
roslaunch Sailboat-Simulation ocean_gazebo_laser.launch
cd Sailboat-Simulation/Gazebo/usv_gazebo/src
python send_drive_cmd.py

1. 
cd helper/pre_processing/
python pub_tf_tree_not_sync.py

2.
cd segmentation/scripts
python segmentation_ros.py

3.
roslaunch tracking all.launch

4.
roslaunch tracking view.launch