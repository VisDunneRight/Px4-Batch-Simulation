FROM ubuntu:20.04

ARG DEBIAN_FRONTEND=noninteractive
ARG USER_NAME=ardupilot
ARG FIRMWARE_DIR=Firmware
ARG USER_UID=1000
ARG USER_GID=1000
RUN groupadd ${USER_NAME} --gid ${USER_GID}\
    && useradd -l -m ${USER_NAME} -u ${USER_UID} -g ${USER_GID} -s /bin/bash

RUN apt-get update && apt-get install -y \
    git \
    lsb-release \
    software-properties-common \
    # ADD THIS HERE
    python3 \
    python3-pip \
    python3-dev \
    python3-numpy \
    python3-opencv \
    python3-wxgtk4.0 \
    python3-matplotlib \
    python3-lxml \
    python3-pyqt5 \
    curl \
    sudo \
    xvfb \
    x11vnc \
    && rm -rf /var/lib/apt/lists/*

# RUN useradd -m ardupilot && echo "ardupilot:ardupilot" | chpasswd && adduser ardupilot sudo
# USER ardupilot
WORKDIR /home/ardupilot/

# Clone ardupilot repo
RUN git clone https://github.com/ArduPilot/ardupilot.git ${FIRMWARE_DIR}
WORKDIR /home/ardupilot/${FIRMWARE_DIR}

# Initialize submodules
RUN git submodule update --init --recursive

###################### Install ArduPilot ###############################################

RUN echo "ardupilot ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USER_NAME}
RUN chmod 0440 /etc/sudoers.d/${USER_NAME}

# RUN apt-get install libffi-dev
RUN chown -R ${USER_NAME}:${USER_NAME} ./

USER ${USER_NAME}
ENV SKIP_AP_EXT_ENV=1 SKIP_AP_GRAPHIC_ENV=1 SKIP_AP_COV_ENV=1 SKIP_AP_GIT_CHECK=1
RUN /Tools/environment_install/install-prereqs-ubuntu.sh -y

# add waf alias to ardupilot waf to .ardupilot_env
RUN echo "alias waf=\"/${USER_NAME}/waf\"" >> ~/.ardupilot_env

# Check that local/bin are in PATH for pip --user installed package
RUN echo "if [ -d \"\$HOME/.local/bin\" ] ; then\nPATH=\"\$HOME/.local/bin:\$PATH\"\nfi" >> ~/.ardupilot_env

############################################################################################

# Install JSBSim
# RUN sudo apt-get update && sudo apt-get install -y libtool libtool-bin g++ automake autoconf libexpat1-dev libglib2.0-dev
# RUN git clone https://github.com/JSBSim-Team/jsbsim.git
# WORKDIR /home/ardupilot/jsbsim
# RUN ./autogen.sh --enable-libraries
# RUN make
# RUN sudo make install
# WORKDIR /home/ardupilot/ardupilot

# Install SITL and MAVProxy
RUN sudo pip3 install wheel future lxml
RUN sudo pip3 install MAVProxy

# Install wget
RUN sudo apt-get update && sudo apt-get install -y wget

# Install Gazebo
RUN sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
RUN sudo apt-get update && sudo apt-get install -y gazebo11 libgazebo11-dev
RUN echo "source /usr/share/gazebo-11/setup.sh" >> ~/.bashrc

# Install ROS Noetic
RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN sudo apt-get update && sudo apt-get install -y ros-noetic-desktop-full
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN sudo apt-get install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
RUN sudo rosdep init
RUN rosdep update

CMD ["bash"]
