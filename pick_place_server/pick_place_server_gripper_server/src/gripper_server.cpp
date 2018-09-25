#include <actionlib/server/simple_action_server.h>
#include <pick_place_server_gripper_server/GripAction.h>

#include <gazebo_ros_link_attacher/Attach.h>
#include <robotiq_85_msgs/GripperCmd.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <pick_place_server_gripper_server/GripAction.h>

typedef actionlib::SimpleActionServer<pick_place_server_gripper_server::GripAction> Server;
typedef boost::function<void(pick_place_server_gripper_server::GripGoalConstPtr&)> ExecuteCallback;

void executeSim(const pick_place_server_gripper_server::GripGoalConstPtr& goal, Server* as, ros::Publisher* gripper_command_pub)
{
    ROS_INFO("Executing sim action");

    ros::NodeHandle n;

    gazebo_ros_link_attacher::Attach srv;
    srv.request.model_name_1 = goal->object;
    srv.request.link_name_1 = "link";
    srv.request.model_name_2 = "robot";
    srv.request.link_name_2 = "wrist_3_link";
    ros::ServiceClient client;


    trajectory_msgs::JointTrajectory joint_traj_msg;
    trajectory_msgs::JointTrajectoryPoint joint_trajectory_point;
    joint_traj_msg.header.stamp = ros::Time::now();
    joint_traj_msg.joint_names.push_back(std::string("robotiq_85_left_knuckle_joint"));

    if (goal->open)
    {
        client = n.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/detach");

        joint_trajectory_point.positions.push_back(0.0);
    }
    else
    {
        client = n.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach");

        joint_trajectory_point.positions.push_back(goal->sim_close_grip);
    }

    pick_place_server_gripper_server::GripResult result;

    joint_trajectory_point.time_from_start = ros::Duration(1);
    joint_traj_msg.points.push_back(joint_trajectory_point);

    gripper_command_pub->publish(joint_traj_msg);

    if (client.call(srv))
    {
        ROS_INFO("Successfully attached");
        result.success = 0;
        as->setSucceeded(result);
    }
    else
    {
        ROS_INFO("Could not attach");
        result.success = 1;
        as->setAborted(result);
    }
}

void executeReal(const pick_place_server_gripper_server::GripGoalConstPtr& goal, Server* as, ros::Publisher* gripper_command_pub)
{
    ROS_INFO("Executing real action");

    if (goal->open)
    {
        robotiq_85_msgs::GripperCmd open_grip;
        open_grip.position = 0.85;
        open_grip.speed = 0.02;
        open_grip.force = 1.0;
        gripper_command_pub->publish(open_grip);
    }
    else
    {
        robotiq_85_msgs::GripperCmd close_grip;
        close_grip.position = 0.0;
        close_grip.speed = 0.02;
        close_grip.force = 1.0;
        gripper_command_pub->publish(close_grip);
    }

    pick_place_server_gripper_server::GripResult result;
    result.success = 0;
    as->setSucceeded(result);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "grip_server");
    ros::NodeHandle pnh("~");
    ros::NodeHandle n;
    void (*executeFunc)(const pick_place_server_gripper_server::GripGoalConstPtr& goal, Server* as, ros::Publisher* gripper_command_pub);

    bool sim;
    pnh.param<bool>("sim", sim, true);

    ros::Publisher gripper_command_pub;

    if (sim)
    {
        ROS_INFO("Starting sim action server");
        executeFunc = &executeSim;
        gripper_command_pub = n.advertise<trajectory_msgs::JointTrajectory>("/gripper/command", 10);
    }
    else
    {
        ROS_INFO("Starting real action server");
        executeFunc = &executeReal;
        gripper_command_pub = n.advertise<robotiq_85_msgs::GripperCmd>("/right_gripper/cmd", 10);
    }


    Server server(n, "grip", boost::bind(executeFunc, _1, &server, &gripper_command_pub), false);
    server.start();
    ros::spin();
    return 0;
}
