# Simulation of Autonomous Landing of a Multirotor UAV on a Moving Platform

![Example Landing](landing.gif)

## Project Overview
This project involves the development of a simulation model for the autonomous landing of a multirotor unmanned aerial vehicle on a moving landing platform. The simulation utilizes Aruco fractal marker pose estimation and ultra-wideband technology for precise landing. The control system includes two PID controllers for horizontal positioning along the XY axes, and a log-polynomial controller for smooth descent along the Z axis. This work is part of my bachelor thesis.

## ROS Package Structure
### Config

- `camera_calibration/` - Contains calibration files for the camera used in the simulation.
- `drone_fsm.yaml` - Configuration for the drone finite state machine.
- `landing_controller.yaml` - Configuration for the landing controller.
- `aruco_estimator.yaml` - Configuration for Aruco marker estimation.
- `uwb_lqr_vanc_estimator.yaml` - Configuration for UWB estimation.
- `uwb_simulator.yaml` - Configuration for the UWB simulator.
- `ugv_control.yaml` - Configuration for the unmanned ground vehicle.

### Launch
- `drone_fsm.launch` - Launch file for the drone finite state machine.
- `landing_controller.launch` - Launch file for the landing controller.
- `aruco_estimator.launch` - Launch file for the Aruco marker estimator.
- `uwb_lqr_vanc_estimator.launch` - Launch file for the UWB estimator.
- `uwb_simulator.launch` - Launch file for the UWB simulator.
- `spawn_husky.launch` - Launch file for spawning the Husky model.
- `spawn_iris.launch` - Launch file for spawning the Iris model.
- `ugv_control.launch` - Launch file for UGV control.
- `uav_landing_sim.launch` - Main launch file for the UAV landing simulation.


### Messages
- `UWBRange.msg` - Message definition for UWB range

### Scripts

#### Drone

- `drone_fsm.py` - Script for the drone finite state machine.
- `landing_controller.py` - Script for the landing controller.

#### Positioning

- `aruco_estimator.py` - Script for Aruco marker estimation.
- `uwb_lqr_vanc_estimator.py` - Script for UWB estimation.

#### Sensor Simulation

- `uwb_simulator.py` - Script for the UWB simulator.

#### UGV

- `ugv_control.py` - Script for UGV control.

### Python Modules

##### `drone/`

- `offboard.py` – Handles offboard control of the drone via MAVROS.

##### `env_disturbances/`

- `env_disturbances.py` – Simulates environmental disturbances affecting the positioning.

##### `uwb_interpolation/`

- `rmse_interpolation.py` – Computes RMSE-based interpolation of UWB signals.
- `sigma_interpolation.py` – Interpolates UWB data based on signal variance or noise characteristics.


### Worlds

- `sim_windy.world` - Gazebo world file for simulating windy conditions.
- `sim,word` - General Gazebo world file.

## Simulation Details
The simulation employs the following technologies for accurate landing:

- **Aruco Fractal Marker Pose Estimation:** Utilizes Aruco fractal markers to determine the precise position and orientation of the landing platform.
- **Ultra-Wideband Technology:** Uses and simulates UWB signals for accurate distance measurement and positioning.

During the simulation, the UAV first takes off to a predefined altitude specified in the configuration file, and then proceeds to fly to a predefined GPS position where it hovers while waiting for the landing platform. The platform moves along an elliptical trajectory and starts its motion 25 seconds after the simulation begins. Once the UAV is able to acquire a valid position estimate (via Aruco or UWB), the landing process is initiated. If positioning is lost during descent, the UAV will attempt to ascend after a short delay. If this does not help regain positioning, the UAV will return to the predefined GPS position and continue waiting for the platform in order to attempt landing again.

## Installation and usage
You can install and run the project in two ways:
### 1. Using Docker
A Docker setup is available for easier and more reproducible deployment.  
**Note:** Make sure you have `docker` and `docker-compose` installed on your system.

To run the simulation using Docker Compose:
1. Clone the repository:
```bash
git clone https://github.com/dimianx/uav_landing_sim
cd uav_landing_sim/docker
```
2. Build the image:
```bash
docker-compose build
```
3. Run the container:
```bash
docker-compose up
```
4. Find the container ID of the running simulation container (look for the image dimianx/uav_landing_sim:devel):
```bash
docker container ls
```
5. Access the container's shell:
```bash
docker exec -it <container_id> bash
```
Replace `<container_id>` with the actual ID from the previous command.
6. Build the project inside the container:
```bash
catkin build
```

7. Source the environment:
```bash
source /entrypoint.sh
```
8. Launch the simulation using `roslaunch` with desired arguments:
```bash
roslaunch uav_landing_sim uav_landing_sim.launch positioning:="..." world_name:="..."
```
**Arguments:**
- `positioning`: either `"aruco"` (default, using fractal Aruco markers) or `"uwb"` (using ultra-wideband positioning)
- `world_name`: name of the Gazebo world to load; either `"sim.world"` or `"sim_windy.world"` (default: `"sim.world"`)

If no arguments are provided, the default behavior is:
- Positioning via fractal **Aruco** markers (`positioning:="aruco"`)
- World: **sim.world** (`world_name:="sim.world"`)

> **Note:**   On the first launch, Gazebo may take some time to download required models from the internet. It might look like Gazebo is frozen — just wait a bit until everything loads.

### 2. Manual Installation
> This project requires **ROS Noetic** and **Ubuntu 20.04**.  Make sure ROS is properly installed and sourced before proceeding.
1. Clone the repository:
```bash
git clone https://github.com/dimianx/uav_landing_sim
cd uav_landing_sim 
```
2. Create a new ROS workspace:
```
mkdir -p ./workspace/src
```
2. Move the `uav_landing_sim` ROS package folder into created workspace
```bash
mv -v ./uav_landing_sim ./workspace/src
```
3. Clone PX4-Autopilot (v1.12.3 is supported)
```bash
git clone https://github.com/PX4/PX4-Autopilot.git --branch v1.12.3 --recursive
```
4. Install the necessary [UAV models](https://github.com/boris-gu/fpv-drone)
```bash
git clone https://github.com/boris-gu/fpv-drone
cd fpv-drone
bash ./setup.sh ../PX4-Autopilot
```
5. Build PX4 SITL
```bash
cd ../PX4-Autopilot
bash ./Tools/setup/ubuntu.sh
DONT_RUN=1 make px4_sitl_default gazebo
```
> **Note:**  By default, the Ninja build system used by PX4 utilizes all available CPU cores for parallel compilation. This may lead to an **out-of-memory** error during the `sitl_gazebo` target build.
>
>To avoid this, limit the number of parallel jobs by modifying the PX4 build configuration **before** running `make`:
>```bash
>sed -i '/ExternalProject_Add(sitl_gazebo/,/)/s/BUILD_COMMAND.*/BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> -- -j 1/' \
>    platforms/posix/cmake/sitl_target.cmake
>```
>
>This will force the SITL Gazebo build to use only one thread, reducing memory usage.


6. Build and install the [ArUco library](https://sourceforge.net/projects/aruco)
```
cd ../
wget https://deac-riga.dl.sourceforge.net/project/aruco/3.1.12/aruco-3.1.12.zip?viasf=1 -O aruco.zip
unzip aruco.zip
cd aruco-3.1.12
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install
```

7. Install Python dependencies:
```bash
cd ../../
pip3 install -r ./workspace/src/uav_landing_sim/requirements.txt
```

> **Note:**  
> If you encounter the following error during build or runtime:
> ```
> fatal error: opencv2/opencv.hpp: No such file or directory
> ```
> it may be caused by OpenCV headers being located in `/usr/include/opencv4/opencv2` instead of `/usr/include/opencv2`.
>
>To fix this, create a symbolic link:
>```bash
>sudo ln -sfv /usr/include/opencv4/opencv2 /usr/include/
>```

8. Install ROS dependencies:
```bash
sudo apt install python3-rosdep python3-catkin-tools
cd ./workspace
rosdep install --from-paths src --ignore-src -r -y
```

9. Install GeographicLib datasets:
```bash
sudo bash /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh
```
10. Build the project
```
source /opt/ros/noetic/setup.bash
catkin build 
```
11. Update your shell environment:
```bash
source /opt/ros/noetic/setup.bash
source <PATH TO THE UAV_LANDING_SIM_ROOT>/PX4-Autopilot/Tools/setup_gazebo.bash <PATH TO THE UAV_LANDING_SIM_ROOT>/PX4-Autopilot <PATH TO THE UAV_LANDING_SIM_ROOT>/PX4-Autopilot/build/px4_sitl_default
source <PATH TO THE UAV_LANDING_SIM_ROOT>/workspace/devel/setup.bash
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:<PATH TO THE UAV_LANDING_SIM_ROOT>/uav_landing_sim/PX4-Autopilot
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:<PATH TO THE UAV_LANDING_SIM_ROOT/uav_landing_sim/PX4-Autopilot/Tools/sitl_gazebo
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:<PATH TO THE UAV_LANDING_SIM_ROOT>/workspace/src/uav_landing_sim/models
```
>**Note:** Replace `<PATH TO THE UAV_LANDING_SIM_ROOT>` with the actual path to the root of your `uav_landing_sim` project directory.

>**Note:** You need to update your environment **every time** after building with `catkin build`.To avoid doing this manually each time, you can add the above lines to your `~/.bashrc` file.

12. Launch the simulation using `roslaunch` with desired arguments:
```bash
roslaunch uav_landing_sim uav_landing_sim.launch positioning:="..." world_name:="..."
```
**Arguments:**
- `positioning`: either `"aruco"` (default, using fractal Aruco markers) or `"uwb"` (using ultra-wideband positioning)
- `world_name`: name of the Gazebo world to load; either `"sim.world"` or `"sim_windy.world"` (default: `"sim.world"`)

If no arguments are provided, the default behavior is:
- Positioning via fractal **Aruco** markers (`positioning:="aruco"`)
- World: **sim.world** (`world_name:="sim.world"`)

>**Note:** On the first launch, Gazebo may take some time to download required models from the internet. It might look like Gazebo is frozen — just wait a bit until everything loads.

# References
- [Bachelor thesis](https://elib.spbstu.ru/dl/3/2023/vr/vr23-3642.pdf/en/info)
- [Anikin Dmitry, et al. "Autonomous landing algorithm for UAV on a mobile robotic platform with a fractal marker." International Conference on Interactive Collaborative Robotics. Cham: Springer Nature Switzerland, 2023.](https://doi.org/10.1007/978-3-031-43111-1_32)
- [Ryabinov A. V., Saveliev A. I., Anikin D. A. Modeling the Effect of External Actions on the Process of Automated Landing of a Quadcopter UAV on a Moving Platform Using Technical Vision //Automatic Control and Computer Sciences. – 2024. – Т. 58. – №. 7. – С. 957-968.](https://doi.org/10.3103/S014641162470038X)