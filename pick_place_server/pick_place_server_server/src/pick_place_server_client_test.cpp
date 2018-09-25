#include <actionlib/client/simple_action_client.h>
#include <pick_place_server_server/PickPlaceAction.h>
#include <ros/ros.h>

typedef actionlib::SimpleActionClient<pick_place_server_server::PickPlaceAction> Client;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pick_place_server_client_test");
    ROS_INFO("Setting up client");
    Client client("pickplace", true);
    ROS_INFO("Starting pick place server");

    while (ros::ok()) {
	client.waitForServer();

	pick_place_server_server::PickPlaceGoal goal;
	goal.pick = "Can1";
	goal.pick_traj_name = "Soup Can Top Grasp";
	goal.pick_traj_waypoints = { 1 };
	//goal.pick_traj_waypoints = { 0, 1 };
	goal.place_frame = "base_link";
	goal.place_traj_name = "Target Set Object On";
	//goal.place_traj_waypoints = {0};
	ROS_INFO("Setting goal");

	ROS_INFO("Sending goal");
	client.sendGoal(goal);
	ROS_INFO("Waiting on result");
	client.waitForResult(ros::Duration(300.0));

	if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
	    ROS_INFO("Execution successful!");
	} else {
	    ROS_INFO("Execution failed");
	}
    }
    return 0;
}
