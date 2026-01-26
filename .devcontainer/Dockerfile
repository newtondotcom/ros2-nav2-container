FROM --platform=linux/${TARGETARCH} osrf/ros:jazzy-desktop-full

ARG USERNAME=rosuser
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
RUN if id -u $USER_UID; then userdel `id -un $USER_UID`; fi

# Create the user
RUN \
  groupadd --gid $USER_GID $USERNAME \
  && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME

# Install sudo and add user to sudoers
RUN echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
  && chmod 0440 /etc/sudoers.d/$USERNAME

# Install dependencies and some tools
RUN \
  --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  apt-get update \
  && apt-get upgrade -y \
  && apt-get install --no-install-recommends -y \
    ros-${ROS_DISTRO}-navigation2 ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-turtlebot3* \
    ros-${ROS_DISTRO}-nav2-minimal-tb3* ros-${ROS_DISTRO}-robot-localization ros-${ROS_DISTRO}-mapviz ros-${ROS_DISTRO}-mapviz-plugins ros-${ROS_DISTRO}-tile-map ros-${ROS_DISTRO}-teleop-twist-keyboard ros-${ROS_DISTRO}-multires-image

# Source ROS environment automatically
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash # ROS_SETUP_SCRIPT" >> /home/$USERNAME/.bashrc
RUN echo "export GZ_SIM_RESOURCE_PATH=/workspaces/ros2-devcontainer/src/nav2_gps_waypoint_follower_demo/models:/workspaces/ros2-devcontainer/src/nav2_gps_waypoint_follower_demo/worlds:$USERNAME/.gz/fuel/fuel.gazebosim.org:/usr/share/gz:/usr/share/gazebo" >> /home/$USERNAME/.bashrc

ENV SHELL=/bin/bash
USER $USERNAME

CMD ["/bin/bash"]
