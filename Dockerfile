FROM ros:humble

# Install required dependencies
RUN apt update && apt install -y \
    python3-pip \
    python3-tk \
    terminator 

RUN apt update && apt upgrade -y && apt install -y \
    ros-humble-ros-base \
    ros-humble-turtlebot3* \
    ros-humble-gazebo-* \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

RUN apt update && apt upgrade -y && apt-get install ros-humble-rqt-graph -y


RUN pip3 install \
    speechrecognition \
    vosk

RUN apt install wget

RUN mkdir -p ~/model && cd ~/model && \
    wget https://alphacephei.com/vosk/models/vosk-model-small-en-us-0.15.tar.gz && \
    tar -xzvf vosk-model-small-en-us-0.15.tar.gz --strip-components=1 && \
    rm vosk-model-small-en-us-0.15.tar.gz

RUN mkdir ~/tf_install && cd ~/tf_install && \
    git clone https://github.com/DLu/tf_transformations.git

RUN cd ~/tf_install/tf_transformations && \
    pip3 install .


RUN pip3 install transforms3d


# Set up TurtleBot3 workspace
RUN mkdir -p ~/turtlebot3_ws/src && cd ~/turtlebot3_ws/src && \
    git clone -b humble https://github.com/ROBOTIS-GIT/DynamixelSDK.git && \
    git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git && \
    git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3.git && \
    git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    cd ~/turtlebot3_ws && \
    colcon build --symlink-install"


# RUN apt update && mkdir -p ~/interplanetar_ws/src && cd ~/interplanetar_ws/src && \
#     rm -rf turtlebot3_interplanetar && \
#     git clone https://github.com/easynotsoeasy/turtlebot3_interplanetar.git


# RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
#     cd ~/interplanetar_ws && \
#     colcon build --symlink-install"



# Configure environment variables
RUN echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc && \
    # echo 'source ~/interplanetar_ws/install/setup.bash' >> ~/.bashrc && \
    echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc && \
    echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc && \
    echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc


RUN apt update && apt install -y \
    portaudio19-dev \
    python3-pyaudio

RUN pip3 install pyaudio


CMD ["bash"]
