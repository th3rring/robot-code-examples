# Kavraki Lab Demos (Fetch). How to Use:

These packages can be installed through [kavrakilab-env](https://github.com/KavrakiLab/kavrakilab_env).

## 0. Installing (checking out) Fetch simulation tests

Before installing any new package, update your installed packages:
```
kavrakilab-get update
```

To install (check out) the Fetch tests in simulation:
```
kavrakilab-get install ros-test_fetch_simulation_bringup
```

To install (check out) the Fetch tests in the real robot:
```
kavrakilab-get install ros-test_fetch_hardware_bringup
```

To build the new packages in either case:
```
kavrakilab-make
```

## 1. Localization tests:

To test the different localization options, you should start RViz:```test-fetch-desktop-rviz```, and move the Fetch around: ```test-fetch-desktop-base-teleop```. Map tf must remain consistent, while odom may drift.

There are three options for localizing the Fetch:

### a. SLAM using Gmapping (simulation and real Fetch):

To start the Fetch in simulation (Gazebo) and Gmapping:
```
test-fetch-simulation-full world_name:=test_zone.sdf mapping:=true
```

Or to start the real Fetch and Gmapping:
```
sudo service robot stop # This will stop most of the Fetch's modules started at boot
test-fetch-hardware-full mapping:=true
```

Move around. Map tf must remain consistent, while odom may drift.

### b. Using an existing map and localizing with AMCL:

To start the Fetch in simulation (Gazebo) and AMCL using an existing map:
```
test-fetch-simulation-full world_name:=test_zone.sdf mapping:=false map_file:=simulation_map_gmapping
```

Or to start the real Fetch and AMCL using an existing map:
```
sudo service robot stop # This will stop most of the Fetch's modules started at boot
test-fetch-hardware-full mapping:=false map_file:=kavrakilab_map_gmapping
```

### c. Using Vicon data as global reference:

To start the Fetch in simulation (Gazebo), the ```vicon_bridge_emulator```, and ```vicon_localization```:
```
test-fetch-simulation-full world_name:=test_zone.sdf mapping:=false vicon_bridge:=true vicon_localization:=true
```

In simulation we use the ```vicon_bridge_emulator```. This node emulates the Vicon pose and tf information. To add any Gazebo object into the ```vicon_bridge_emulator``` (for instance table2):
```
rosservice call /vicon/add_vicon_object "object: 'table2'"
```
Likewise, to remove any Gazebo object into the ```vicon_bridge_emulator``` (for instance table2):
```
rosservice call /vicon/remove_vicon_object "object: 'table2'"
```

On to start the real Fetch, the ```vicon_bridge```, and ```vicon_localization```:
```
sudo service robot stop # This will stop most of the Fetch's modules started at boot
test-fetch-hardware-full mapping:=false vicon_bridge:=true vicon_localization:=true
```

In the real Fetch we use the ```vicon_bridge``` (further details can be found in the [wiki](https://github.com/KavrakiLab/workspaces/wiki/How-to-Use-the-Vicon-System)). 
Note: check that ```test_fetch_common_bringup\launch\vicon_localization.launch``` has the correct IP addrees of the Vicon Windows computer.


## 2. Planning Demo:

### a. Using Vicon data as global reference:

To start the Fetch in simulation (Gazebo), the ```vicon_bridge_emulator```, ```vicon_localization```, and the ```vicon_object_server```:
```
test-fetch-simulation-full world_name:=demo_fetch.sdf mapping:=false vicon_bridge:=true vicon_localization:=true vicon_bridge_params_file:=pick_place_vicon_localization.yaml moveit:=true
roslaunch vicon_object_server_server server.launch scene:=fetch_vicon_demo_sim.yaml base_frame:=map
```

Or to start the real Fetch, the ```vicon_bridge```, ```vicon_localization```, and the ```vicon_object_server```:
```
sudo service robot stop # This will stop most of the Fetch's modules started at boot
test-fetch-hardware-full mapping:=false vicon_bridge:=true vicon_localization:=true vicon_bridge_params_file:=pick_place_vicon_localization.yaml moveit:=true
roslaunch vicon_object_server_server server.launch scene:=fetch_vicon_demo.yaml base_frame:=map
```

### b. Using Gmapping as global reference:

To start the Fetch in simulation (Gazebo), the ```vicon_bridge_emulator```, and the ```vicon_object_server```:
```
test-fetch-simulation-full world_name:=demo_fetch.sdf mapping:=true vicon_bridge:=true vicon_bridge_params_file:=pick_place_gmapping_localization.yaml moveit:=true
roslaunch vicon_object_server_server server.launch package_path:=test_fetch_common_bringup/parameters/vicon_object_server/scenes scene:=fetch_vicon_demo_sim.yaml base_frame:=world
```

Or to start the real Fetch, the ```vicon_bridge```, and the ```vicon_object_server```:
```
sudo service robot stop # This will stop most of the Fetch's modules started at boot
test-fetch-hardware-full mapping:=true vicon_bridge:=true vicon_bridge_params_file:=pick_place_gmapping_localization.yaml moveit:=true
roslaunch vicon_object_server_server server.launch scene:=fetch_vicon_demo.yaml base_frame:=world
```

There are two different demos.

Once consist in moving the gripper on top of the can. To start that demo ```fetch_vicon_demo``` in either simulation or with the real Fetch:
```
rosrun test_fetch_common_bringup fetch_vicon_demo _fetch_vicon_topic:=/vicon/fetch/fetch #(for simulation, or replace the topic with /vicon/Fetch/Fetch for the real Fetch and the real Vicon -Yes it is case-sensitive!)
```

The other consist in picking and placing the can in a predefined location of the table. To start that demo ```fetch_vicon_pick_demo``` in either simulation or with the real Fetch:
```
rosrun test_fetch_common_bringup fetch_vicon_pick_demo _fetch_vicon_topic:=/vicon/fetch/fetch #(for simulation, or replace the topic with /vicon/Fetch/Fetch for the real Fetch and the real Vicon -Yes it is case-sensitive!)
```
