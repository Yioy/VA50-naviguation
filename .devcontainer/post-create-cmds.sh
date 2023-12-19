#!/bin/bash

echo 'source /opt/ros/noetic/setup.bash' >> ~/.bashrc

# if devel/setup.bash does not exist, then compile the workspace with catkin build
if [ ! -f devel/setup.bash ]; then
    echo "Building workspace..."
    catkin build
    echo "Building and installing cython binaries..."
    ./build_circulation.sh cython
fi
echo 'source $(pwd)/devel/setup.bash' >> ~/.bashrc

echo "alias utacsim='/simulator/UTACSimulator.x86_64'" >> ~/.bashrc