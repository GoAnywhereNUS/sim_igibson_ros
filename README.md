# sim_igibson_ros
## Description
This repository provides instructions and code to set up an iGibson environment with a ROS interface and ROS navigation stack for testing out navigation algorithms or collecting data.

## Setup
1. `git clone https://github.com/StanfordVL/iGibson --recursive`
2. Build the iGibson ROS Docker image. Run `cd iGibson/docker/igibson-ros && ./build.sh`
3. Start a container, _with a mounted volume_ to allow files to be transferred into the container. For these instructions, we assume that the host directory is mounted at `/home/files`.
4. Overwrite all items in the catkin workspace with the contents of the `igibson-ros` folder from this repository: `rm /opt/catkin_ws/src/igibson-ros/* && cp -r /home/files/sim_igibson_ros/igibson-ros/* /opt/catkin_ws/src/igibson-ros/`
5. Transfer required datasets and meshes to the folder `/opt/iGibson/igibson/data/g_datasets`
6. Edit the config files to specify the desired sensor output.
7. Run `source /opt/ros/noetic/setup.bash && source /opt/catkin_ws/devel/setup.bash`
8. Run simulation with RViz: `cd /opt/catkin_ws/src/igibson-ros/launch && roslaunch turtlebot_custom_nav.launch`

## Datasets
iGibson is compatible with a wide variety of datasets, which can be found here: https://stanfordvl.github.io/iGibson/dataset.html. It is necessary to download the _iGibson_ datasets, as the original Gibson Env datasets are not compatible with the latest simulator.

Some of the datasets (e.g. Stanford 2D-3D-S) may come without generated occupancy grid floor plans. This is needed for the ROS navigation stack's global planner to compute paths, and may be important for your use case. We can generate occupancy grids of each floor in the dataset with the following method:

1. `cd /opt/catkin_ws/src/igibson-ros`
2. `python generate_trav_map.py --scene_names <path/to/scene1> <path/to/scene2> ...`
3. _Optional_: The generated occupancy grids may have a flipped coordinate frame. This can be checked with the RViz visualisation. To flip the grid, run `python flip_trav_map --img_path <path/to/image/to/flip>`