#!/bin/bash
#
# This file is part of the uav_landing_sim distribution (https://github.com/dimianx/uav_landing_sim).
# Copyright (c) 2025 Dmitry Anikin.
#
# This program is free software: you can redistribute it and/or modify  
# it under the terms of the GNU General Public License as published by  
# the Free Software Foundation, version 3.
#
# This program is distributed in the hope that it will be useful, but 
# WITHOUT ANY WARRANTY; without even the implied warranty of 
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
# General Public License for more details.
#
# You should have received a copy of the GNU General Public License 
# along with this program. If not, see <http://www.gnu.org/licenses/>.

# Source ROS
source /opt/ros/${ROS_DISTRO}/setup.bash
echo "Sourced ROS  ${ROS_DISTRO}"

# Add PX4 Gazebo modules to ROS_PACKAGE_PATH
if [ -f /uav_landing_sim/PX4-Autopilot/Tools/setup_gazebo.bash ]
then 
  source /uav_landing_sim/PX4-Autopilot/Tools/setup_gazebo.bash /uav_landing_sim/PX4-Autopilot /uav_landing_sim/PX4-Autopilot/build/px4_sitl_default
  echo "Sourced PX4 Gazebo modules"
fi

# Source the base workspace, if built
if [ -f ${ROS_WORKSPACE}/devel/setup.bash ]
then
  source ${ROS_WORKSPACE}/devel/setup.bash
  echo "Sourced ${ROS_WORKSPACE} workspace"
fi

# Add PX4 modules to ROS_PACKAGE_PATH
if [ -f /uav_landing_sim/PX4-Autopilot/Tools/setup_gazebo.bash ]
then 
  export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/uav_landing_sim/PX4-Autopilot
  export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/uav_landing_sim/PX4-Autopilot/Tools/sitl_gazebo
fi

# Add project's models to GAZEBO_MODE_PATH
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:${ROS_WORKSPACE}/src/uav_landing_sim/models

# Execute the command passed into this entrypoint
exec "$@"