FROM ubuntu:20.04
WORKDIR /ardupilot

ARG DEBIAN_FRONTEND=noninteractive
ARG USER_NAME=ardupilot
ARG FIRMWARE_DIR=Firmware
ARG USER_UID=1000
ARG USER_GID=1000
RUN groupadd ${USER_NAME} --gid ${USER_GID}\
    && useradd -l -m ${USER_NAME} -u ${USER_UID} -g ${USER_GID} -s /bin/bash

RUN apt-get update && apt update && apt-get install --no-install-recommends -y \
    lsb-release \
    sudo \
    tzdata \
    bash-completion 

RUN apt-get -y install git
RUN apt install -y python3
RUN apt install -y python3-pip

RUN sudo apt install -y python3-lxml libxml2-utils
RUN pip3 install dronekit dronekit-sitl
RUN pip3 install pymavlink==2.4.8

# Download REPO
RUN cd /${USER_NAME} && git clone https://github.com/ArduPilot/ardupilot.git  ${FIRMWARE_DIR}
RUN cd  /${USER_NAME}/${FIRMWARE_DIR} && git submodule update --init --recursive

# COPY /ardupilot/Tools/environment_install/install-prereqs-ubuntu.sh /ardupilot/Tools/environment_install/
# COPY /ardupilot/Tools/completion /ardupilot/Tools/completion/

# Create non root user for pip
ENV USER=${USER_NAME}


###################### Install ArduPilot ######################

RUN echo "ardupilot ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USER_NAME}
RUN chmod 0440 /etc/sudoers.d/${USER_NAME}
# RUN apt-get install libffi-dev

RUN chown -R ${USER_NAME}:${USER_NAME} /${USER_NAME}

USER ${USER_NAME}
ENV SKIP_AP_EXT_ENV=1 SKIP_AP_GRAPHIC_ENV=1 SKIP_AP_COV_ENV=1 SKIP_AP_GIT_CHECK=1
RUN ${FIRMWARE_DIR}/Tools/environment_install/install-prereqs-ubuntu.sh -y

# add waf alias to ardupilot waf to .ardupilot_env
RUN echo "alias waf=\"/${USER_NAME}/waf\"" >> ~/.ardupilot_env

# Check that local/bin are in PATH for pip --user installed package
RUN echo "if [ -d \"\$HOME/.local/bin\" ] ; then\nPATH=\"\$HOME/.local/bin:\$PATH\"\nfi" >> ~/.ardupilot_env

# Create entrypoint as docker cannot do shell substitution correctly
RUN export ARDUPILOT_ENTRYPOINT="/home/${USER_NAME}/ardupilot_entrypoint.sh" \
    && echo "#!/bin/bash" > $ARDUPILOT_ENTRYPOINT \
    && echo "set -e" >> $ARDUPILOT_ENTRYPOINT \
    && echo "source /home/${USER_NAME}/.ardupilot_env" >> $ARDUPILOT_ENTRYPOINT \
    && echo 'exec "$@"' >> $ARDUPILOT_ENTRYPOINT \
    && chmod +x $ARDUPILOT_ENTRYPOINT \
    && sudo mv $ARDUPILOT_ENTRYPOINT /ardupilot_entrypoint.sh

# ####################### Install Gazebo ######################
# RUN sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' \
#     && wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add - \
#     && sudo apt update

# # RUN sudo apt install gazebo9 libgazebo9-dev
# RUN sudo apt-get install -y gazebo11 libgazebo11-dev

# ####################### Gazebo Plugin Installation ######################
# RUN cd /${USER_NAME} && git clone https://github.com/khancyr/ardupilot_gazebo \
#     && cd ardupilot_gazebo && mkdir build && cd build && cmake .. && make -j4

# # Install Gazebo
# RUN sudo echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc
# RUN sudo echo 'export GAZEBO_MODEL_PATH=/ardupilot/ardupilot_gazebo/models' >> ~/.bashrc
# RUN . ~/.bashrc

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

####################### Clean Up ######################
# Set the buildlogs directory into /tmp as other directory aren't accessible
ENV BUILDLOGS=/tmp/buildlogs

RUN sudo apt-get clean \
    && sudo rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

ENV CCACHE_MAXSIZE=1G

# Build ArduCopter sitl 
RUN cd ${FIRMWARE_DIR} && ./waf configure --board sitl && ./waf copter

# # ENTRYPOINT ["/ardupilot_entrypoint.sh"]
CMD ["bash"]