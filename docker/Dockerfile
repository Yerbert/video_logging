FROM osrf/ros:melodic-desktop-full

# Dev tools
RUN apt-get -y update && apt-get install -y \
    x11-apps \
    python-pip \
    python-vcstool \
    build-essential \
    git \
    ros-melodic-catkin \
    python-catkin-tools

RUN pip install pynput
# Make the prompt a little nicer
RUN echo "PS1='${debian_chroot:+($debian_chroot)}\u@:\w\$ '" >> /etc/bash.bashrc  

# Project specific dependencies
ENV CATKIN_WS=/catkin_ws
ENV SOURCE_DIR=${CATKIN_WS}/src/video_logging
COPY ./ar_vr.rosinstall ${SOURCE_DIR}/ar_vr.rosinstall
WORKDIR ${CATKIN_WS}/src
RUN vcs import < ${SOURCE_DIR}/ar_vr.rosinstall

COPY ./package.xml ${SOURCE_DIR}/package.xml
WORKDIR ${CATKIN_WS}
RUN rosdep update && rosdep install --from-paths src --ignore-src -r -y

# Build the project
COPY . ${SOURCE_DIR}
RUN /bin/bash -c '. /opt/ros/$ROS_DISTRO/setup.bash; cd /${CATKIN_WS}; catkin build'
WORKDIR ${SOURCE_DIR}

# Add source commands to bashrc
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /etc/bash.bashrc
RUN echo "source ${CATKIN_WS}/devel/setup.bash" >> /etc/bash.bashrc

ENTRYPOINT /ros_entrypoint.sh
CMD /bin/bash