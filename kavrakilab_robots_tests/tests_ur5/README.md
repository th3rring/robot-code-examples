# Kavraki Lab Demos (UR5). How to Use:

These packages can be installed through [kavrakilab-env](https://github.com/KavrakiLab/kavrakilab_env).

## 0. Installing (checking out) UR5 simulation tests

Before installing any new package, update your installed packages:
```
kavrakilab-get update
```

To install (check out) the UR5 tests in simulation:
```
kavrakilab-get install ros-test_ur5_simulation_bringup
```

To install (check out) the UR5 tests in the real robot:
```
kavrakilab-get install ros-test_ur5_hardware_bringup
```

To build the new packages in either case:
```
kavrakilab-make
```


## Planning Demo:

### a. Using Vicon data as global reference:

To start the UR5 in simulation (Gazebo), use the ```vicon_bridge_emulator``` and the ```vicon_object_server``` and run the following launch commands in order in separate terminals:
```
test-ur5-simulation-start-gazebo
test-ur5-simulation-start-moveit
test-ur5-simulation-start-vicon-object-server
test-ur5-desktop-rviz
```
These commands will launch Gazebo, start the controllers and MoveIt!, the vicon object server, the vicon bridge emulator, and rviz. To actually run the demo, run the following command:
```
rosrun test_ur5_common_bringup ur5_vicon_pick_demo 
```
The demo uses the cube object as a safety marker. In order to start the demo, you will have to move it off the table in Gazebo.

(Under Development) Or to start the real UR5, use the ```vicon_bridge``` and the ```vicon_object_server```:
```
sudo service robot stop # This will stop most of the UR5's modules started at boot
test-ur5-hardware-full mapping:=false vicon_bridge:=true vicon_localization:=true vicon_bridge_params_file:=pick_place_vicon_localization.yaml moveit:=true
roslaunch vicon_object_server_server server.launch scene:=ur5_vicon_demo.yaml base_frame:=map
```

There are two different demos.

(Under Development) Once consist in moving the gripper on top of the can. To start that demo ```ur5_vicon_demo``` in either simulation or with the real ur5:
```
rosrun test_ur5_common_bringup ur5_vicon_demo _ur5_vicon_topic:=/vicon/ur5/ur5 #(for simulation, or replace the topic with /vicon/ur5/ur5 for the real ur5 and the real Vicon -Yes it is case-sensitive!)
```

(Under Development) The other consist in picking and placing the can in a predefined location of the table. To start that demo ```ur5_vicon_pick_demo``` in either simulation or with the real ur5:
```
rosrun test_ur5_common_bringup ur5_vicon_pick_demo _ur5_vicon_topic:=/vicon/ur5/ur5 #(for simulation, or replace the topic with /vicon/ur5/ur5 for the real ur5 and the real Vicon -Yes it is case-sensitive!)
```
