FROM osrf/ros:noetic-desktop-full-focal

# install python modules
RUN apt-get update \
    && apt-get install -y \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*
COPY requirements.txt /build/requirements.txt
RUN python3 -m pip install -r /build/requirements.txt


# Install apt dependencies
RUN apt-get update \
    && apt-get install -y \
    nano \
    git-all \
    python3-pip \
    ros-noetic-ros-numpy \
    python3-catkin-tools \
    && rm -rf /var/lib/apt/lists/*

# Rosdep
RUN rosdep update

# Create a non-root user
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config


# Set up sudo
RUN apt-get update \
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && rm -rf /var/lib/apt/lists/*


# Required to run the simulator GUI with the GPU
ENV  __NV_PRIME_RENDER_OFFLOAD=1
ENV __GLX_VENDOR_LIBRARY_NAME=nvidia
ENV NVIDIA_DRIVER_CAPABILITIES=all

USER ros

RUN pip3 install roboflow==1.1.7
RUN pip3 install easyocr==1.7.1
# that one is installed as a roboflow dependecy, and replaces the existing installation...
RUN pip3 uninstall -y opencv-python-headless

CMD ["bash"]