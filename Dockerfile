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
        python3-catkin-tools \
        python3-osrf-pycommon && \
        rm -rf /var/lib/apt/lists/*

USER developer
    RUN mkdir -p /home/developer/mobile_robot_fleet
    RUN echo "source /opt/ros/noetic/setup.bash" >> /home/developer/.bashrc
    WORKDIR /home/developer/mobile_robot_fleet

ENTRYPOINT ["/bin/bash", "-c"]
