ARG ROS_DISTRO=humble
FROM ros:$ROS_DISTRO

ARG USERNAME=jack
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME
RUN apt-get clean && apt-get update
# Add sudo support
RUN apt-get install -y sudo
RUN echo $USERNAME ALL=\(ALL\) NOPASSWD: ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip python3-vcstools clangd clang-format socat apt-utils

USER $USERNAME
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
ADD ./src /src-for-rosdep
SHELL ["/bin/bash", "-c"]
USER root
RUN addgroup realtime \
    && usermod -a -G realtime $USERNAME \
    && echo "@realtime soft rtprio 99" >> /etc/security/limits.conf \
	&& echo "@realtime soft priority 99" >> /etc/security/limits.conf \
	&& echo "@realtime soft memlock 102400" >> /etc/security/limits.conf \
	&& echo "@realtime hard rtprio 99" >> /etc/security/limits.conf \
	&& echo "@realtime hard priority 99" >> /etc/security/limits.conf \
	&& echo "@realtime hard memlock 102400" >> /etc/security/limits.conf

USER $USERNAME
# Then to build, run 'colcon build --mixin release' in /proj/ws/src
RUN echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc \
    && echo "export ROS_DISTRO=$ROS_DISTRO" >> ~/.bashrc \
    && echo 'source /proj/ws_$ROS_DISTRO/install/setup.bash' >> ~/.bashrc

RUN rosdep update --rosdistro $ROS_DISTRO \
	&& rosdep install -r -y --rosdistro $ROS_DISTRO --from-paths /src-for-rosdep
	# ros-$ROS_DISTRO-kortex-bringup \
	# ros-$ROS_DISTRO-kortex-driver \
	# ros-$ROS_DISTRO-kortex-description \
# RUN sudo apt-get install -y \
# 	ros-$ROS_DISTRO-ament-cmake \
# 	ros-$ROS_DISTRO-rclcpp \
# 	ros-$ROS_DISTRO-rclcpp-components \
# 	ros-$ROS_DISTRO-ros-gz-bridge \
# 	ros-$ROS_DISTRO-ros-gz-sim \
# 	ros-$ROS_DISTRO-moveit-servo \
# 	ros-$ROS_DISTRO-robot-state-publisher \
# 	ros-$ROS_DISTRO-tf2-ros \
# 	ros-$ROS_DISTRO-tf2-eigen \
# 	ros-$ROS_DISTRO-ros2launch \
# 	ros-$ROS_DISTRO-ament-lint-auto \
# 	ros-$ROS_DISTRO-ament-lint-common

# RUN source /opt/ros/$ROS_DISTRO/setup.bash \
# 	&& colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && colcon mixin update default
RUN sudo apt-get install -y qtwayland5 ros-$ROS_DISTRO-moveit-setup-assistant  # ros-$ROS_DISTRO-rqt-controller-manager ros-$ROS_DISTRO-ros2controlcli ros-$ROS_DISTRO-ros2controlcli
ENV SHELL /bin/bash

RUN sudo apt-get install -y ros-humble-moveit-visual-tools
ADD ../jaka_ros2 /jaka_ros2
RUN echo "source /jaka_ros2/src/setup.bash" >> ~/.bashrc

# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************

# [Optional] Set the default user. Omit if you want to keep the default as root.
CMD ["/bin/bash"]
