FROM ros:noetic-ros-base

USER root
    RUN useradd -m developer && \
        echo "developer:123" | chpasswd && \
        adduser developer sudo
    RUN apt-get update && apt-get install -y \
        cmake \
        git \
        python3-catkin-tools \
        python3-osrf-pycommon && \
        rm -rf /var/lib/apt/lists/*

USER developer
    RUN mkdir -p /home/developer/catkin_ws/src
    RUN echo "source /opt/ros/noetic/setup.bash" >> /home/developer/.bashrc
    WORKDIR /home/developer/catkin_ws

ENTRYPOINT ["/bin/bash", "-c"]
