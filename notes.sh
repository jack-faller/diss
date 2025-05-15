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
# sometimes you need to add this idk why it doesn't work in the docker file
colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && colcon mixin update default
# to build
colcon build --mixin debug && ros2 launch project project.launch.py

ros2 launch kinova_gen3_lite_moveit_config robot.launch.py robot_ip:=192.168.1.10 use_fake_hardware:=true
ros2 launch project project.launch.py

ros2 launch kortex_bringup kortex_sim_control.launch.py \
  sim_gazebo:=true \
  launch_rviz:=false \
  robot_type:=gen3_lite dof:=6

ros2 launch kortex_bringup gen3_lite.launch.py robot_ip:=yyy.yyy.yyy.yyy use_fake_hardware:=true
