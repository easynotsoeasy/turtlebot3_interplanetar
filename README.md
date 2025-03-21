# ROS 2 Interplanetar Project

## Overview
This guideline is for **Ubuntu**. It has not been tested on macOS or Windows.

There are two versions of the Docker image for this project:
- **`ros2_interplanetar_base`**: Contains ROS 2 Humble and other necessary dependencies to run the project but **DOES NOT** include TurtleBot3 or Gazebo.
- **`ros2_interplanetar_full`**: Includes everything necessary, including TurtleBot3 and Gazebo.

> I recommend using the `ros2_interplanetar_full` version because I've tested it thoroughly. Installing Gazebo and TurtleBot3 in `ros2_interplanetar_base` can be a hassle sometimes.

## Install and Run Docker

First, install Docker if you don't already have it: [Docker Installation Guide](https://docs.docker.com/engine/install/ubuntu/).

> **Note:** I have Docker set up with `usermod`, so I don't need to use `sudo` before each Docker command. You may need to use `sudo`.

### Pull and Run the Docker Image
Open a terminal and run:

#### Pull the Docker Image
```sh
docker pull easyyyy/ros2_interplanetar_base  # Base version
docker pull easyyyy/ros2_interplanetar_full  # Full version
```

#### Verify the Image
```sh
docker image ls
```

#### Enable GUI Support for Docker
```sh
xhost +local:docker
```

#### Run the Docker Container
```sh
sudo docker run -it \
    --env DISPLAY=$DISPLAY \
    --volume /tmp/.X11-unix:/tmp/.X11-unix \
    --device /dev/snd \
    --env PULSE_SERVER=unix:${XDG_RUNTIME_DIR}/pulse/native \
    -v ${XDG_RUNTIME_DIR}/pulse/native:${XDG_RUNTIME_DIR}/pulse/native \
    -v ~/.config/pulse/cookie:/root/.config/pulse/cookie \
    easyyyy/ros2_interplanetar_full
```
> This ensures that Docker has permission for **GUI and Audio Support**.

## Inside the Docker Container

### Run Terminator for Multiple Terminals
```sh
terminator
```
> Terminator helps to run multiple nodes at the same time.

### Pull the Source Code
```sh
mkdir -p ~/interplanetar_ws/src && cd ~/interplanetar_ws/src && \
    rm -rf turtlebot3_interplanetar && \
    git clone https://github.com/easynotsoeasy/turtlebot3_interplanetar.git
```

### Build the Project
```sh
/bin/bash -c "source /opt/ros/humble/setup.bash && \
    cd ~/interplanetar_ws && \
    colcon build --symlink-install"
```

### Setup the Environment
```sh
echo 'source ~/interplanetar_ws/install/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

## Running the Nodes
Move to the source directory (only necessary to run `speaker_node`):
```sh
cd ~/
```
> **Note:** `speaker_node` runs correctly only from the root directory due to an issue with the Vosk model path. All other nodes work from anywhere.

### Splitting the Terminal in Terminator
- **CTRL + SHIFT + E**: Split terminal vertically
- **CTRL + SHIFT + O**: Split terminal horizontally

### Run the Nodes
```sh
ros2 run turtlebot3_interplanetar speaker_node_text
ros2 run turtlebot3_interplanetar speaker_node
ros2 run turtlebot3_interplanetar energy_node
ros2 run turtlebot3_interplanetar control_node
```

### Run the Control GUI
```sh
ros2 run turtlebot3_interplanetar control_gui
```

### Launch TurtleBot3 in Gazebo
```sh
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

### View the Node Graph
```sh
rqt_graph
