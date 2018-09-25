#include <iostream>
#include <vector>

#include <cmath>
#include <cstdlib>
#include <string>

// ROS stuff
#include <ros/ros.h>

#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>

#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>

#include <vicon_object_client.h>
#include <vicon_object_server.h>

#include <geometry_msgs/PoseStamped.h>
#include <moveit/robot_state/robot_state.h>

#include <geometry_msgs/TransformStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <vicon_object_server_msgs/TrajGetWaypoint.h>

#include <actionlib/client/simple_action_client.h>
#include <robotiq_85_msgs/GripperCmd.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <test_ur5_common_bringup/GripAction.h>

typedef actionlib::SimpleActionClient<test_ur5_common_bringup::GripAction> Client;

using namespace vicon_object_server;

class Ur5ViconDemo
{
public:
    // Constructor
    Ur5ViconDemo();
    // Callback for getting game block position
    void gameBlockPosCallback(const ros::TimerEvent& event);
    //! Function for waiting for user confirmation
    void waitForUserConfirmation(const std::string& next_task);
    //! Function to close the gripper
    void closeGripper(const std::string& object_name,
                      moveit::planning_interface::MoveGroupInterface& move_group);
    //! Function to open the gripper
    void openGripper(const std::string& object_name,
                     moveit::planning_interface::MoveGroupInterface& move_group);
    //! Run
    void run();

private:
    // ROS
    ros::NodeHandle node_handler_, local_nh_;
    ros::Subscriber game_block_pos_sub_;
    ros::Publisher gripper_commander_;
    // ros::Publisher gripper_command_pub_;

    // Objects
    std::vector<double> last_game_block_pos_;
    std::string safety_obj_vicon_topic_;

    bool replan_;

    // TF Listener and Buffer
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Gripper Server
    Client grip_client_;
};

Ur5ViconDemo::Ur5ViconDemo()
  : replan_(false)
  , local_nh_("~")
  , safety_obj_vicon_topic_("/vicon/game_block/game_block")
  , tf_listener_(tf_buffer_)
  , grip_client_("grip", true)
{
    last_game_block_pos_.resize(3);

    //=======================================================================
    // Get parameters
    //=======================================================================
    local_nh_.param("safety_obj_vicon_topic", safety_obj_vicon_topic_, safety_obj_vicon_topic_);

    //=======================================================================
    // Subscribers
    //=======================================================================
    // game_block_pos_sub_ = node_handler_.subscribe(safety_obj_vicon_topic_, 1,
    //                                               &Ur5ViconDemo::gameBlockPosSubCallback, this);

    //=======================================================================
    // Publishers
    //=======================================================================
    // gripper_commander_ =
    //     node_handler_.advertise<robotiq_85_msgs::GripperCmd>("/right_gripper/cmd", 10);
    // grip_client_.waitForServer();

    // gripper_command_pub_ =
    // node_handler_.advertise<trajectory_msgs::JointTrajectory>("/gripper/command", 10);
}

void Ur5ViconDemo::closeGripper(const std::string& object_name,
                                moveit::planning_interface::MoveGroupInterface& move_group)
{
    test_ur5_common_bringup::GripGoal goal;
    goal.open = false;
    goal.object = object_name;
    goal.sim_close_grip = 0.1;

    grip_client_.sendGoal(goal);
    grip_client_.waitForResult(ros::Duration(1.0));

    if (grip_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Grip closed");
    }

    move_group.attachObject(
        object_name, "ee_link",
        {"robotiq_85_base_link", "robotiq_85_left_finger_link", "robotiq_85_left_finger_tip_link",
         "robotiq_85_left_inner_knuckle_link", "robotiq_85_left_knuckle_link",
         "robotiq_85_right_finger_link", "robotiq_85_right_finger_tip_link",
         "robotiq_85_right_inner_knuckle_link", "robotiq_85_right_knuckle_link"});

    sleep(1.0);
}

void Ur5ViconDemo::openGripper(const std::string& object_name,
                               moveit::planning_interface::MoveGroupInterface& move_group)
{
    move_group.detachObject(object_name);

    test_ur5_common_bringup::GripGoal goal;
    goal.object = object_name;
    goal.open = true;

    grip_client_.sendGoal(goal);
    grip_client_.waitForResult(ros::Duration(1.0));

    if (grip_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Grip open");
    }

    sleep(1.0);
}

void Ur5ViconDemo::gameBlockPosCallback(const ros::TimerEvent& event)
{
    geometry_msgs::TransformStamped pos_msg;

    try
    {
        // Got duration amount from vicon object server main.cpp
        pos_msg = tf_buffer_.lookupTransform("base_link", "vicon_object_server/demo_cube",
                                             ros::Time(0), ros::Duration(0.1));
    }
    catch (tf2::LookupException& ex)
    {
        ROS_ERROR("VICONObjectServer::lookup(%s, %s): %s", "/base_link",
                  "/vicon_object_server/demo_cube", ex.what());
        throw Exception(1, "TF Lookup Failure");
    }

    // Going to get game block position in frame of base_link with tf_listener
    last_game_block_pos_[0] = pos_msg.transform.translation.x;
    last_game_block_pos_[1] = pos_msg.transform.translation.y;
    last_game_block_pos_[2] = pos_msg.transform.translation.z;

    double dist = sqrt(pow(last_game_block_pos_[0], 2.0) + pow(last_game_block_pos_[1], 2.0) +
                       pow(last_game_block_pos_[2], 2.0));
    if (dist < 1.40)
        replan_ = true;
    else
        replan_ = false;
}

void Ur5ViconDemo::run()
{
    const std::string TRAJECTORIES_CAN = "Soup Can Top Grasp";
    const std::string TRAJECTORIES_TARGET = "Target Set Object On";
    ros::AsyncSpinner spinner(4);
    spinner.start();

    // Set up moveit interface
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::planning_interface::MoveGroupInterface move_group("manipulator");

    // Taken from my existing code, verify against existing demo
    moveit_msgs::PlanningScene planning_scene;
    ros::Publisher planning_scene_diff_publisher =
        node_handler_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

    // Set up gripper commander and prevent any model collisions
    vicon_object_server::VICONObjectClient vicon_object_client(true);
    Eigen::Affine3d pre_grasp = vicon_object_client.trajGetWaypoint(TRAJECTORIES_CAN, 0);
    Eigen::Affine3d grasp = vicon_object_client.trajGetWaypoint(TRAJECTORIES_CAN, 1);
    Eigen::Affine3d place = vicon_object_client.trajGetWaypoint(TRAJECTORIES_TARGET, 0);

    // Create a timer to call the tf listener callback
    ros::Timer timer =
        node_handler_.createTimer(ros::Duration(0.1), &Ur5ViconDemo::gameBlockPosCallback, this);

    //=======================================================================
    // Replanning if needed
    //=======================================================================
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        if (replan_)
        {
            //"Waiting the user to move away."
            while (replan_ && ros::ok())
                loop_rate.sleep();

            // Pregrasp
            geometry_msgs::PoseStamped target_pose_pre_grasp;
            tf::poseEigenToMsg(pre_grasp, target_pose_pre_grasp.pose);
            target_pose_pre_grasp.header.frame_id = "vicon_object_server/Can1";
            move_group.setPoseTarget(target_pose_pre_grasp);

            auto success = move_group.plan(my_plan);
            if (success == moveit::planning_interface::MoveItErrorCode::SUCCESS)
                move_group.execute(my_plan);
            loop_rate.sleep();
            sleep(1.0);

            // Grasp
            move_group.clearPoseTargets();
            geometry_msgs::PoseStamped target_pose_grasp;
            tf::poseEigenToMsg(grasp, target_pose_grasp.pose);
            target_pose_grasp.header.frame_id = "vicon_object_server/Can1";
            move_group.setPoseTarget(target_pose_grasp);

            loop_rate.sleep();
            success = move_group.plan(my_plan);
            if (success == moveit::planning_interface::MoveItErrorCode::SUCCESS)
                move_group.execute(my_plan);
            sleep(1.0);

            closeGripper("Can1", move_group);

            vicon_object_client.objectDisable("Can1");
            vicon_object_client.objectDisable("Target");
            sleep(1.0);

            // Pregrasp
            tf::poseEigenToMsg(pre_grasp, target_pose_pre_grasp.pose);
            target_pose_pre_grasp.header.frame_id = "vicon_object_server/Can1";
            move_group.setPoseTarget(target_pose_pre_grasp);

            success = move_group.plan(my_plan);
            if (success == moveit::planning_interface::MoveItErrorCode::SUCCESS)
                move_group.execute(my_plan);
            loop_rate.sleep();
            vicon_object_client.objectEnable("Target");
            sleep(1.0);

            // Place
            move_group.clearPoseTargets();
            geometry_msgs::PoseStamped target_pose_place;
            tf::poseEigenToMsg(place, target_pose_place.pose);
            //TODO: You can put base_link as the frame and define the trajectory based off of that
            target_pose_place.header.frame_id = "base_link";
            move_group.setPoseTarget(target_pose_place);

            loop_rate.sleep();
            success = move_group.plan(my_plan);
            if (success == moveit::planning_interface::MoveItErrorCode::SUCCESS)
                move_group.execute(my_plan);

            // Open the gripper and detach the can

            openGripper("Can1", move_group);

            vicon_object_client.objectEnable("Can1");
            sleep(1.0);

            // Pregrasp
            tf::poseEigenToMsg(pre_grasp, target_pose_pre_grasp.pose);
            target_pose_pre_grasp.header.frame_id = "vicon_object_server/Can1";
            move_group.setPoseTarget(target_pose_pre_grasp);

            success = move_group.plan(my_plan);
            if (success == moveit::planning_interface::MoveItErrorCode::SUCCESS)
                move_group.execute(my_plan);
            loop_rate.sleep();
        }
        // std::cout
        sleep(1.0);
        loop_rate.sleep();
        sleep(1.0);
    }

    spinner.stop();
    ros::shutdown();
    sleep(1.0);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ur5_vicon_demo");

    Ur5ViconDemo demo;
    demo.run();

    return 0;
}
