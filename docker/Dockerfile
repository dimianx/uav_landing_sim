# ========================
# Base Image
# ========================
FROM osrf/ros:noetic-desktop-full as base
ENV ROS_DISTRO=noetic
ENV ROS_WORKSPACE=/uav_landing_sim/workspace
SHELL ["/bin/bash", "-c"]

COPY ./docker/entrypoint.sh /
ENTRYPOINT [ "/entrypoint.sh" ]

# ========================
# Devel Image
# ========================
FROM base as devel

ARG USERNAME=devuser
ARG UID=1000
ARG GID=${UID}

RUN apt update && \
    DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends --no-install-suggests \
    wget \
    gdb \
    gdbserver \
    nano \
    git \
    build-essential \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /uav_landing_sim/workspace/src
WORKDIR /uav_landing_sim
COPY ./uav_landing_sim ./workspace/src/uav_landing_sim

# Clone PX4-Autopilot
WORKDIR /uav_landing_sim
RUN git clone https://github.com/PX4/PX4-Autopilot.git --branch v1.12.3 --recursive

# UAV models
WORKDIR /uav_landing_sim
RUN git clone https://github.com/boris-gu/fpv-drone
RUN cd fpv-drone && bash ./setup.sh /uav_landing_sim/PX4-Autopilot

# Build PX4 SITL
RUN bash ./PX4-Autopilot/Tools/setup/ubuntu.sh && \
    cd PX4-Autopilot && \
    sed -i '/ExternalProject_Add(sitl_gazebo/,/)/s/BUILD_COMMAND.*/BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> -- -j 1/' \
        platforms/posix/cmake/sitl_target.cmake && \
    DONT_RUN=1 make px4_sitl_default gazebo

# ArUco library
WORKDIR /uav_landing_sim
RUN ln -sfv /usr/include/opencv4/opencv2 /usr/include/
RUN wget https://deac-riga.dl.sourceforge.net/project/aruco/3.1.12/aruco-3.1.12.zip?viasf=1 -O aruco.zip && \
    unzip aruco.zip && \
    cd aruco-3.1.12 && \
    mkdir build && \
    cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make -j$(nproc) && \
    make install

# Python deps
WORKDIR /uav_landing_sim/workspace/src/uav_landing_sim
RUN pip3 install -r requirements.txt

# Package deps
WORKDIR /uav_landing_sim/workspace
RUN source /opt/ros/noetic/setup.bash && \
    apt update && \
    DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends --no-install-suggests \
    python3-rosdep \
    python3-catkin-tools \
    x11-apps 

RUN source /opt/ros/noetic/setup.bash && \
    apt update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    catkin build && \
    source /uav_landing_sim/workspace/devel/setup.bash && \
    rm -rf /var/lib/apt/lists/*

# Geographic lib
RUN sudo bash /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh

RUN groupadd --gid $GID $USERNAME \
    && useradd --uid ${GID} --gid ${UID} --create-home ${USERNAME} \
    && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME} \
    && mkdir -p /home/${USERNAME} \
    && chown -R ${UID}:${GID} /home/${USERNAME} 
   
RUN chown -R ${UID}:${GID} /uav_landing_sim

USER ${USERNAME}
RUN echo "source /entrypoint.sh" >> /home/${USERNAME}/.bashrc