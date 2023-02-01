# This will work for Gazebo and JmavSim
FROM ubuntu:focal
ENV PX4_PATH  /PX4-Autopilot
ENV WORKSPACE_DIR /root
ENV FIRMWARE_DIR ${WORKSPACE_DIR}/Firmware

# Update ever apt-get
RUN apt-get update
RUN apt update

# Set working directory
WORKDIR ${WORKSPACE_DIR}

#Intall Python + Python dependencies
RUN apt install -y python3
RUN apt install -y python3-pip
COPY "requirements.txt" "${WORKSPACE_DIR}/requirements.txt"
RUN pip3 install -r "${WORKSPACE_DIR}/requirements.txt"

# # Install ubuntu libraries
RUN apt-get update && apt-get install -y lsb-release && apt-get clean all
RUN apt-get -y install sudo
RUN apt-get install wget -y
RUN apt install gnupg -y && sudo apt -y install gnupg1
RUN apt-get -y install git

# Download Px4 from Github
RUN git clone https://github.com/PX4/PX4-Autopilot.git ${FIRMWARE_DIR}
RUN bash ${FIRMWARE_DIR}/Tools/setup/ubuntu.sh 

# Do pre-build
RUN ["/bin/bash", "-c", " \
    cd ${FIRMWARE_DIR} && \
    DONT_RUN=1 make px4_sitl gazebo"]
    
# RUN ["/bin/bash", "-c", " \
#     cd ${FIRMWARE_DIR} && \
#     DONT_RUN=1 make px4_sitl jmavsim"]

# RUN cd ${FIRMWARE_DIR} && make px4_sitl
EXPOSE 14550/udp 5760/tcp