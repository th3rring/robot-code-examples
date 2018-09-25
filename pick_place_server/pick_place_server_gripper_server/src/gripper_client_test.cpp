#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <pick_place_server_gripper_server/GripAction.h>

typedef actionlib::SimpleActionClient<pick_place_server_gripper_server::GripAction> Client;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gripper_server_client");
    ROS_INFO("Setting up client");
    Client client("grip", true);
    ROS_INFO("Starting real action server");
    client.waitForServer();

    pick_place_server_gripper_server::GripGoal goal;
    goal.open = true;
    goal.object = "cube_sim";
    ROS_INFO("Setting goal");

    ROS_INFO("Sending goal");
    client.sendGoal(goal);
    ROS_INFO("Waiting on result");
    client.waitForResult(ros::Duration(5.0));

    if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Grip closed");
    }
    else
    {
    ROS_INFO("Grip failed");
    }

    return 0;
}
