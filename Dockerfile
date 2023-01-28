FROM ros:noetic-ros-base

USER root
    RUN useradd -m developer && \
        echo "developer:123" | chpasswd && \
        adduser developer sudo
    RUN apt-get update && apt-get install -y \
        clang-format \
        cmake \
        flake8 \
        git \
        python3-tk \
        python3-pip \
        python3-catkin-tools \
        python3-osrf-pycommon && \
        rm -rf /var/lib/apt/lists/*
    RUN apt-get update && apt-get install -y \
        plantuml && \
        rm -rf /var/lib/apt/lists/*
    RUN pip3 install matplotlib

USER developer
    RUN mkdir -p /home/developer/mobile_robot_fleet
    RUN echo "source /opt/ros/noetic/setup.bash" >> /home/developer/.bashrc
    WORKDIR /home/developer/mobile_robot_fleet

ENTRYPOINT ["/bin/bash", "-c"]
