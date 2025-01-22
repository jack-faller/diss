# run the simulator
docker pull universalrobots/ursim_e-series
docker run --rm -it -e ROBOT_MODEL=UR3 -p 5900:5900 -p 6080:6080 --name ursim universalrobots/ursim_e-series
# then use the webui to turn the simulated robot on
# connect to robot/simulator
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=172.17.0.3 launch_rviz:=false initial_joint_controller:=joint_trajectory_controller
# to activate conroller if it is not
# ros2 run rqt_controller_manager rqt_controller_manager
# run moviet
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3e launch_rviz:=true
# to build
colcon build --mixin debug
ros2 launch project project.launch.py
