# ros2-gazebo-satellite-terrain
ROS2 package for spawning and managing satellite image tiles in Gazebo to form a cohesive terrain.

This whole project is an exercise in Gazebo and ROS2 quirks which is why the code looks somewhat convoluted in some places.

![demo of gazebo view](.github/media/demo.png)

### Environment 
- Ubuntu 24.04 / 22.04
- ROS Jazzy / Humble
- Gazebo Harmonic
- Python 3.12

### Installation
Package only tested on Mac M1 using [RoboStack](https://robostack.github.io) and on Ubuntu 24.04.

- Update the `config.yaml` with map size, tile size, zoom, etc.
- Create a `.env` file with the mapbox parameters:
    ```text
    MAPBOX_APIKEY=...
    MAPBOX_USERNAME=...
    MAPBOX_STYLENAME=...
    ```

**Instructions**
- Install repo:  
    ```
    mkdir -p ros2_ws/src
    cd ros2_ws/src
    git clone git@github.com:jekahamster/gazebo_satellite_terrain.git
    cd ~/ros2_ws
    colcon build --symlink-install --packages-select gazebo_satellite_terrain
    ```
- Go to [MapBox](https://www.mapbox.com).
- Open [StyleEditor](https://console.mapbox.com/studio/) and create custom style.
- Create `.env` file with keys `MAPBOX_APIKEY`, `MAPBOX_USERNAME`, `MAPBOX_STYLENAME`.
- Install models for world from [Gazebo](https://github.com/arpg/Gazebo) repo:
    ```
    cd ~/Projects
    git clone https://github.com/arpg/Gazebo.git
    export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/Projects/Gazebo/models
    ```
- Install ros_gz (ROS 2 Jazzy + Gazebo Harmonic):
    ```
    sudo apt install ros-jazzy-ros-gz
    ```
    The launch file sets `GZ_SIM_RESOURCE_PATH` so package models are found. For extra models (e.g. sun, ground_plane), extend it (e.g. `export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/Projects/Gazebo/models`).
    Check that everything is installed correctly:
    ```
    gz sim
    gz sim -r /path/to/ros2_ws/install/gazebo_satellite_terrain/share/gazebo_satellite_terrain/worlds/empty.world
    ```

**Usage**
- To launch the simulation:
    ```
    ros2 launch gazebo_satellite_terrain launch_sim.launch.py
    ```

#### Future work
- Only [MapBox](https://www.mapbox.com) API is currently supported, would be nice to support other backends
- Rectuangular maps are currently not supported on N x N maps, could easily be implemented
- Comprehensive list of dependencies written to `CMakeList.txt` and `package.xml` should be compiled to make install easier