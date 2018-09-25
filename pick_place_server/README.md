# Pick Place Server. How to Use:

These packages create an action server that can pick and place arbitrary objects from the Vicon Object Server.

These packages can be installed through [kavrakilab-env](https://github.com/KavrakiLab/kavrakilab_env).

## 0. Installing (checking out) the main server:

Before installing any new package, update your installed packages:
```
kavrakilab-get update
```

Then, to just checkout the server part without any of the demo run the following:
```
kavrakilab-get install pick_place_server_server
```

If you want to run the block stacking demo that uses the server, run the following command instead of the one above:
```
kavrakilab-get install pick_place_server_demo
```

To build the new packages in either case:
```
kavrakilab-make
```
## 1. Using the server:

This pick and place server functions as an [action server](http://wiki.ros.org/actionlib) that uses the [vicon object server](https://github.com/KavrakiLab/vicon_object_server) and MoveIt!. The basic usage consists of calling the server with a vicon object server trajectory and TF frame for both the pick and place movement. Depending on how the user makes calls to the server, the robot will place either follow an entire trajectory, a specific ordering of waypoints from a trajectory, or a series of user-specified poses.

In order to accept these different ways of specifying place poses, there is a client implementation that removes almost all the boilerplate code. An action server client must still be initilized and passed to the client.

To actually move the gripper, the server implements a gripper server that changes configuration based on whether the user is running in simulation or on the real robot. As such, in the launch file for the pick place server, there is a flag for simulation that must be set to false to run on the real robot.

As a precaution, there is the option to launch the pick place server with a safety cube (similar to the [Fetch demo](https://github.com/KavrakiLab/kavrakilab_robots_tests/tree/master/tests_fetch) in the kavraki lab robot tests). With the cube within 1.4 meters of the robot's base, the robot will not start picking and placing. Only when the cube is removed will the robot execute the procedure. As this is not necessary in all use cases, there is an option to turn off this functionality in the [start_pick_place_server.launch](https://github.com/KavrakiLab/pick_place_server/blob/master/pick_place_server_server/launch/start_pick_place_server.launch) file. If the user wishes to use this, a vicon object server topic for the transform of the cube must be provided.

To see how this all goes together, please take a look at the [pick_place_server_demo](https://github.com/KavrakiLab/pick_place_server/tree/master/pick_place_server_demo) package. There is a block stacking demo that uses the pick place server and the client. There is an example of all three ways of interacting with the server. The package layout in the demo follows the same convention as done in the master branch of the [kavraki lab robot tests](https://github.com/KavrakiLab/kavrakilab_robots_tests) repository.

## 2. Running the demo:

In order to run the pick_place_server_demo, first ensure that they have been installed and built:
```
kavrakilab-get install pick_place_server_demo
kavrakilab-make
```

As was done for both the UR5 demo and Fetch demo in the kavraki lab robot tests master branch, there are several aliases that help start up the demo. Please note that if the following commands are not found, you may have to source the setup.bash file located in ```ros/*distro*/system/devel/``` where ```*distro*``` is the distribution of ros being used. To get started with the demo, run the following 3 commands in order in separate terminals.
```
pick-place-server-demo-simulation-start-gazebo
pick-place-server-demo-simulation-start-moveit
pick-place-server-demo-simulation-start-vicon-object-server
```

There should be a new Gazebo enviroment spawned with a single UR5 and some blocks. To actually run the demo and stack some blocks run the following command in another separate terminal.
```
rosrun pick_place_server_demo_common_bringup block_stack_demo
```
