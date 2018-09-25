#include <iostream>
#include <vector>

#include <cmath>
#include <cstdlib>
#include <string>

// ROS
#include <ros/ros.h>

#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>

#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/robot_state.h>

// Vicon Object Server
#include <vicon_object_client.h>
#include <vicon_object_server.h>
#include <vicon_object_server_msgs/TrajGetWaypoint.h>

// Gripper Server
#include <actionlib/client/simple_action_client.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <pick_place_server_gripper_server/GripAction.h>

typedef actionlib::SimpleActionClient<pick_place_server_gripper_server::GripAction> Client;

// Action Server
#include <actionlib/server/simple_action_server.h>
#include <pick_place_server_server/PickPlaceAction.h>

typedef actionlib::SimpleActionServer<pick_place_server_server::PickPlaceAction> Server;

using namespace vicon_object_server;

class PickPlaceServer {
public:
    // Constructor
    PickPlaceServer();
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
    void execute(const pick_place_server_server::PickPlaceGoalConstPtr& goal);

private:
    // ROS
    ros::NodeHandle node_handler_, local_nh_;
    ros::Subscriber game_block_pos_sub_;
    ros::Publisher gripper_commander_;

    // Objects
    std::vector<double> last_game_block_pos_;
    std::string safety_obj_vicon_topic_;
    bool use_safety_obj_;

    bool replan_;

    // TF Listener and Buffer
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Gripper Server
    Client grip_client_;

    // Action Server
    Server as_;
};

PickPlaceServer::PickPlaceServer()
    : replan_(false)
    , local_nh_("~")
    , safety_obj_vicon_topic_("vicon_object_server/demo_cube")
    , use_safety_obj_(false)
    , tf_listener_(tf_buffer_)
    , grip_client_("grip", true)
    , as_(node_handler_, "pickplace", boost::bind(&PickPlaceServer::execute, this, _1), false)
{
    last_game_block_pos_.resize(3);

    //=======================================================================
    // Get parameters
    //=======================================================================
    local_nh_.param("safety_obj_vicon_topic", safety_obj_vicon_topic_, safety_obj_vicon_topic_);
    local_nh_.param("use_safety_obj", use_safety_obj_, use_safety_obj_);

    //=======================================================================
    // Action Server
    //=======================================================================
    as_.start();
}

void PickPlaceServer::closeGripper(const std::string& object_name,
    moveit::planning_interface::MoveGroupInterface& move_group)
{
    pick_place_server_gripper_server::GripGoal goal;
    goal.open = false;
    goal.object = object_name;
    goal.sim_close_grip = 0.1;

    grip_client_.sendGoal(goal);
    grip_client_.waitForResult(ros::Duration(1.0));

    if (grip_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
	ROS_INFO("Grip closed");
    }

    move_group.attachObject(
	object_name, "ee_link",
	{ "robotiq_85_base_link", "robotiq_85_left_finger_link", "robotiq_85_left_finger_tip_link",
	    "robotiq_85_left_inner_knuckle_link", "robotiq_85_left_knuckle_link",
	    "robotiq_85_right_finger_link", "robotiq_85_right_finger_tip_link",
	    "robotiq_85_right_inner_knuckle_link", "robotiq_85_right_knuckle_link" });

    sleep(1.0);
}

void PickPlaceServer::openGripper(const std::string& object_name,
    moveit::planning_interface::MoveGroupInterface& move_group)
{
    move_group.detachObject(object_name);

    pick_place_server_gripper_server::GripGoal goal;
    goal.object = object_name;
    goal.open = true;

    grip_client_.sendGoal(goal);
    grip_client_.waitForResult(ros::Duration(1.0));

    if (grip_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
	ROS_INFO("Grip open");
    }

    sleep(1.0);
}

void PickPlaceServer::gameBlockPosCallback(const ros::TimerEvent& event)
{
    geometry_msgs::TransformStamped pos_msg;

    try {
	// Got duration amount from vicon object server main.cpp
	pos_msg = tf_buffer_.lookupTransform("base_link", safety_obj_vicon_topic_,
	    ros::Time(0), ros::Duration(0.1));
    } catch (tf2::LookupException& ex) {
	ROS_ERROR("VICONObjectServer::lookup(%s, %s): %s", "/base_link",
	    safety_obj_vicon_topic_, ex.what());
	throw Exception(1, "TF Lookup Failure");
    }

    // Going to get game block position in frame of base_link with tf_listener
    last_game_block_pos_[0] = pos_msg.transform.translation.x;
    last_game_block_pos_[1] = pos_msg.transform.translation.y;
    last_game_block_pos_[2] = pos_msg.transform.translation.z;

    double dist = sqrt(pow(last_game_block_pos_[0], 2.0) + pow(last_game_block_pos_[1], 2.0) + pow(last_game_block_pos_[2], 2.0));
    if (dist < 1.40)
	replan_ = true;
    else
	replan_ = false;
}

void PickPlaceServer::execute(const pick_place_server_server::PickPlaceGoalConstPtr& goal)
{

    std::string pick = goal->pick;
    std::string pick_traj_name = goal->pick_traj_name;
    std::vector<uint16_t> pick_traj_waypoints = goal->pick_traj_waypoints;

    std::string place_frame = goal->place_frame;
    std::string place_traj_name = goal->place_traj_name;
    std::vector<uint16_t> place_traj_waypoints = goal->place_traj_waypoints;

    uint8_t num_place_poses = goal->num_place_poses;
    std::vector<float> place_positions = goal->place_positions;
    std::vector<float> place_orientations = goal->place_orientations;

    //ROS_INFO("Recieved planning request with %zu pick trajs and %zu and place trajs", pick_traj_waypoints.size(), place_traj_waypoints.size());

    // Set up moveit interface
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::planning_interface::MoveGroupInterface move_group("manipulator");

    // Taken from my existing code, verify against existing demo
    moveit_msgs::PlanningScene planning_scene;
    ros::Publisher planning_scene_diff_publisher = node_handler_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

    // Set up gripper commander and prevent any model collisions
    vicon_object_server::VICONObjectClient vicon_object_client(true);

    std::vector<Eigen::Affine3d> pick_trajs;

    // Determine if no trajectory indexes have been provided to follow
    if (pick_traj_waypoints.size() == 0) {

	// If the user hasn't given us a list of waypoints to follow, follow all of them
	Trajectory* pick_traj_info = vicon_object_client.trajInfo(pick_traj_name);
	for (int i = 0; i < pick_traj_info->numWaypoints(); i++)
	    pick_trajs.push_back(vicon_object_client.trajGetWaypoint(pick_traj_name, i));

    } else {

	// If the user has given us waypoints, follow them in order
	for (int i = 0; i < pick_traj_waypoints.size(); i++)
	    pick_trajs.push_back(vicon_object_client.trajGetWaypoint(pick_traj_name, pick_traj_waypoints[i]));
    }

    std::vector<Eigen::Affine3d> place_trajs;

    if (place_positions.size() != 0) {

	// This is a little weird. ROS messages doesn't support multi-dimensional arrays in messages... So to
	// counteract this, I'm placing all the orientations and positions in a single array and specifying
	// how many total poses should be read in.
	for (int i = 0; i < num_place_poses; i++) {

	    // Convert the position and orientation to an affine3d transform
	    Eigen::Vector3d v(place_positions[3*i], place_positions[3*i + 1], place_positions[3*i + 2]);
	    Eigen::Quaterniond q(place_orientations[4*i + 3], place_orientations[4*i], place_orientations[4*i + 1], place_orientations[4*i + 2]);
	    q.normalize();

	    Eigen::Affine3d tf = Eigen::Affine3d::Identity();
	    tf.translation() = v;
	    tf.linear() = q.toRotationMatrix();

	    place_trajs.push_back(tf);
	    ROS_INFO("Hi I'm placing");
	}

    } else {

	// Determine if no trajectory indexes have been provided to follow
	if (place_traj_waypoints.size() == 0) {

	    // If the user hasn't given us a list of waypoints to follow, follow all of them
	    Trajectory* place_traj_info = vicon_object_client.trajInfo(place_traj_name);
	    for (int i = 0; i < place_traj_info->numWaypoints(); i++)
		place_trajs.push_back(vicon_object_client.trajGetWaypoint(place_traj_name, i));
	} else {

	    // If the user has given us waypoints, follow them in order
	    for (int i = 0; i < place_traj_waypoints.size(); i++)
		place_trajs.push_back(vicon_object_client.trajGetWaypoint(place_traj_name, place_traj_waypoints[i]));
	}
    }

    // Create a timer to call the tf listener callback
    ros::Timer timer;
    if (use_safety_obj_) {
	timer = node_handler_.createTimer(ros::Duration(0.1), &PickPlaceServer::gameBlockPosCallback, this);
	replan_ = true;
    } else {
	replan_ = false;
    }

    // Make a result
    pick_place_server_server::PickPlaceResult result;
    result.err_traj_name = "None";
    result.err_traj_waypoint = 0;

    //"Waiting the user to move away."
    while (replan_ && ros::ok())
	;

    // Follow all pick trajectories and then close the gripper
    for (int i = 0; i < pick_trajs.size(); i++) {

	geometry_msgs::PoseStamped target_pose_pick;
	tf::poseEigenToMsg(pick_trajs[i], target_pose_pick.pose);
	target_pose_pick.header.frame_id = "vicon_object_server/" + pick;
	move_group.setPoseTarget(target_pose_pick);

	auto success = move_group.plan(my_plan);
	if (success == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
	    move_group.execute(my_plan);
	} else {
	    ROS_INFO("Failed to plan for pick");
	    result.err_traj_name = pick_traj_name;
	    result.err_traj_waypoint = i;
	    as_.setAborted(result);
	    return;
	}
	sleep(1.0);
    }

    // Close gripper on object
    closeGripper(pick, move_group);
    vicon_object_client.objectDisable(pick);
    sleep(1.0);

    // Follow all place trajectories and then open the gripper
    for (int i = 0; i < place_trajs.size(); i++) {

	geometry_msgs::PoseStamped target_pose_place;
	tf::poseEigenToMsg(place_trajs[i], target_pose_place.pose);
	target_pose_place.header.frame_id = place_frame;
	move_group.setPoseTarget(target_pose_place);

	auto success = move_group.plan(my_plan);
	if (success == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
	    move_group.execute(my_plan);
	} else {
	    ROS_INFO("Failed to plan for place");
	    result.err_traj_name = place_traj_name;
	    result.err_traj_waypoint = i;
	    as_.setAborted(result);
	    return;
	}
	sleep(1.0);
    }

    // Open gripper
    openGripper(pick, move_group);
    vicon_object_client.objectEnable(pick);
    sleep(1.0);

    as_.setSucceeded(result);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pick_place_server");
    PickPlaceServer server_obj;
    ros::spin();
    return 0;
}
