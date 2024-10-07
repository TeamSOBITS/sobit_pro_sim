#!/bin/bash

echo "╔══╣ Setup: SOBIT PRO Simulations (STARTING) ╠══╗"


# Keep track of the current directory
DIR=`pwd`
cd ..

# Download required packages for SOBIT PRO
ros_packages=(
    "sobit_pro"
)

# Clone all packages
for ((i = 0; i < ${#ros_packages[@]}; i++)) {
    echo "Clonning: ${ros_packages[i]}"
    git clone https://github.com/TeamSOBITS/${ros_packages[i]}.git

    # Check if install.sh exists in each package
    if [ -f ${ros_packages[i]}/install.sh ]; then
        echo "Running install.sh in ${ros_packages[i]}."
        cd ${ros_packages[i]}
        bash install.sh
        cd ..
    fi
}

# Download ROS packages
sudo apt-get update
sudo apt-get install -y \
    ros-$ROS_DISTRO-roboticsgroup-upatras-gazebo-plugins

# Go back to previous directory
cd ${DIR}


echo "╚══╣ Setup: SOBIT PRO Simulations (FINISHED) ╠══╝"
