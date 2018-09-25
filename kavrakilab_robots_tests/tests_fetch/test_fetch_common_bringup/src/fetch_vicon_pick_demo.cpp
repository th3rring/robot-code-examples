#include <iostream>
#include <vector>

#include <cmath>
#include <cstdlib>
#include <string>

// ROS stuff
#include <ros/ros.h>

#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>

#include <vicon_object_client.h>

#include <geometry_msgs/PoseStamped.h>
#include <moveit/robot_state/robot_state.h>

#include <control_msgs/GripperCommandAction.h>
#include <control_msgs/PointHeadAction.h>
#include <control_msgs/PointHeadGoal.h>
#include <geometry_msgs/TransformStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <vicon_object_server_msgs/TrajGetWaypoint.h>

#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<control_msgs::GripperCommandAction> gripper_action_client_t;
typedef actionlib::SimpleActionClient<control_msgs::PointHeadAction> point_head_action_client_t;

//!  FetchViconDemo class.
/*!
 * Fetch Vicon Demo.
 *
*/
class FetchViconDemo
{
public:
    //! Constructor
    FetchViconDemo();
    //! Callback for getting game block position
    void gameBlockPosSubCallback(const geometry_msgs::TransformStampedConstPtr& pos_msg);
    //! adds a padding box above the base to avoid self-collision
    void addBaseBox();
    //! Callback for getting fetch position
    void fetchPosSubCallback(const geometry_msgs::TransformStampedConstPtr& pos_msg);
    //! Function for waiting for user confirmation
    void waitForUserConfirmation(const std::string& next_task);
    //! Run
    void run();

private:
    // ROS
    ros::NodeHandle node_handler_, local_nh_;
    ros::Subscriber game_block_pos_sub_, fetch_pos_sub_;

    // Objects
    std::vector<double> last_game_block_pos_, last_fetch_pos_;
    std::string safety_obj_vicon_topic_, fetch_vicon_topic_;

    bool replan_, fetch_pos_available_;

    // Controller action
    boost::shared_ptr<gripper_action_client_t> gripper_action_client_;
    boost::shared_ptr<point_head_action_client_t> point_head_action_client_;
};

FetchViconDemo::FetchViconDemo()
  : replan_(false)
  , fetch_pos_available_(false)
  , local_nh_("~")
  , safety_obj_vicon_topic_("/vicon/game_block/game_block")
  , fetch_vicon_topic_("/vicon/fetch/fetch")
{
    last_game_block_pos_.resize(3);
    last_fetch_pos_.resize(3);

    //=======================================================================
    // Get parameters
    //=======================================================================
    local_nh_.param("safety_obj_vicon_topic", safety_obj_vicon_topic_, safety_obj_vicon_topic_);
    local_nh_.param("fetch_vicon_topic", fetch_vicon_topic_, fetch_vicon_topic_);

    //=======================================================================
    // Action lib
    //=======================================================================
    std::string action_name = "gripper_controller/gripper_action";
    gripper_action_client_.reset(new gripper_action_client_t(action_name, true));
    if (!gripper_action_client_->waitForServer(ros::Duration(2.0)))
        ROS_ERROR("%s may not be connected.", action_name.c_str());

    action_name = "head_controller/point_head";
    point_head_action_client_.reset(new point_head_action_client_t(action_name, true));
    if (!point_head_action_client_->waitForServer(ros::Duration(2.0)))
        ROS_ERROR("%s may not be connected.", action_name.c_str());

    //=======================================================================
    // Subscribers
    //=======================================================================
    game_block_pos_sub_ = node_handler_.subscribe(safety_obj_vicon_topic_, 1,
                                                  &FetchViconDemo::gameBlockPosSubCallback, this);
    fetch_pos_sub_ =
        node_handler_.subscribe(fetch_vicon_topic_, 1, &FetchViconDemo::fetchPosSubCallback, this);
}

void FetchViconDemo::addBaseBox()
{
    moveit_msgs::PlanningScene planning_scene_collision;
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = "/base_link";
    collision_object.id = "keep_out";
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.2;
    primitive.dimensions[1] = 0.5;
    primitive.dimensions[2] = 0.05;
    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1;
    box_pose.position.x = 0.15;
    box_pose.position.y = 0;
    box_pose.position.z = 0.375;
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    planning_scene_collision.world.collision_objects.push_back(collision_object);
    planning_scene_collision.is_diff = true;
    ros::Publisher planning_scene_diff_publisher =
        node_handler_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    while (planning_scene_diff_publisher.getNumSubscribers() < 1)
    {
        ros::WallDuration sleep_t(0.5);
        sleep_t.sleep();
    }
    // Publish the added object
    planning_scene_diff_publisher.publish(planning_scene_collision);
}

void FetchViconDemo::gameBlockPosSubCallback(const geometry_msgs::TransformStampedConstPtr& pos_msg)
{
    last_game_block_pos_[0] = pos_msg->transform.translation.x;
    last_game_block_pos_[1] = pos_msg->transform.translation.y;
    last_game_block_pos_[2] = pos_msg->transform.translation.z;

    if (fetch_pos_available_)
    {
        double dist = sqrt(pow(last_game_block_pos_[0] - last_fetch_pos_[0], 2.0) +
                           pow(last_game_block_pos_[1] - last_fetch_pos_[1], 2.0) +
                           pow(last_game_block_pos_[2] - last_fetch_pos_[2], 2.0));
        // std::cout << dist << std::endl;
        if (dist < 1.40)
            replan_ = true;
        else
            replan_ = false;
    }
}

void FetchViconDemo::fetchPosSubCallback(const geometry_msgs::TransformStampedConstPtr& pos_msg)
{
    last_fetch_pos_[0] = pos_msg->transform.translation.x;
    last_fetch_pos_[1] = pos_msg->transform.translation.y;
    last_fetch_pos_[2] = pos_msg->transform.translation.z;

    fetch_pos_available_ = true;
}

void FetchViconDemo::run()
{
    const std::string TRAJECTORIES_CAN = "Soup Can Top Grasp";
    const std::string TRAJECTORIES_TABLE = "Table Set Object On";
    ros::AsyncSpinner spinner(4);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::planning_interface::MoveGroupInterface move_group("arm_with_torso");
    moveit::planning_interface::MoveGroupInterface gripper("gripper");

    move_group.setPlannerId("RRTConnectkConfigDefault");
    auto pcm = planning_scene_monitor::PlanningSceneMonitorPtr(
        new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
    planning_scene_monitor::LockedPlanningSceneRO planning_scene(pcm);

    // This adds the safety box above the fetch base to avoid self-collisions.
    addBaseBox();

    vicon_object_server::VICONObjectClient vicon_object_client(true);
    Eigen::Affine3d pre_grasp = vicon_object_client.trajGetWaypoint(TRAJECTORIES_CAN, 0);
    Eigen::Affine3d grasp = vicon_object_client.trajGetWaypoint(TRAJECTORIES_CAN, 1);
    Eigen::Affine3d place = vicon_object_client.trajGetWaypoint(TRAJECTORIES_TABLE, 0);

    // Open the gripper before starting
    control_msgs::GripperCommandGoal gripper_goal;
    gripper_goal.command.position = 0.115;
    gripper_goal.command.max_effort = 100.0;
    gripper_action_client_->sendGoal(gripper_goal);

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

            // look at the Can1 position
            control_msgs::PointHeadGoal point_head_goal;
            point_head_goal.target.header.stamp = ros::Time::now();
            point_head_goal.target.header.frame_id = "vicon_object_server/Can1";
            point_head_goal.target.point.x = point_head_goal.target.point.y =
                point_head_goal.target.point.z = 0.0;
            point_head_goal.min_duration = ros::Duration(1.0);
            point_head_action_client_->sendGoal(point_head_goal);
            point_head_action_client_->waitForResult();

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
            // Close the gripper and attach the can
            gripper_goal.command.position = 0.0;
            gripper_action_client_->sendGoal(gripper_goal);

            sleep(2.0);
            move_group.attachObject("Can1", "r_gripper_finger_link",
                                    {"r_gripper_finger_link", "l_gripper_finger_link", "gripper_"
                                                                                       "link"});
            //
            vicon_object_client.objectDisable("Can1");
            vicon_object_client.objectDisable("table1");
            sleep(1.0);

            // look at the Can1 position
            point_head_goal.target.header.stamp = ros::Time::now();
            point_head_goal.target.header.frame_id = "vicon_object_server/table1";
            point_head_goal.target.point.x = point_head_goal.target.point.y =
                point_head_goal.target.point.z = 0.0;
            point_head_goal.min_duration = ros::Duration(1.0);
            point_head_action_client_->sendGoal(point_head_goal);
            point_head_action_client_->waitForResult();

            // Pregrasp
            tf::poseEigenToMsg(pre_grasp, target_pose_pre_grasp.pose);
            target_pose_pre_grasp.header.frame_id = "vicon_object_server/Can1";
            move_group.setPoseTarget(target_pose_pre_grasp);

            success = move_group.plan(my_plan);
            if (success == moveit::planning_interface::MoveItErrorCode::SUCCESS)
                move_group.execute(my_plan);
            loop_rate.sleep();
            vicon_object_client.objectEnable("table1");
            sleep(1.0);

            // look at the Table position
            point_head_goal.target.header.stamp = ros::Time::now();
            point_head_goal.target.header.frame_id = "vicon_object_server/table1";
            point_head_goal.target.point.x = point_head_goal.target.point.y =
                point_head_goal.target.point.z = 0.0;
            point_head_goal.min_duration = ros::Duration(1.0);
            point_head_action_client_->sendGoal(point_head_goal);
            point_head_action_client_->waitForResult();

            // Place
            move_group.clearPoseTargets();
            geometry_msgs::PoseStamped target_pose_place;
            tf::poseEigenToMsg(place, target_pose_place.pose);
            target_pose_place.header.frame_id = "vicon_object_server/table1";
            move_group.setPoseTarget(target_pose_place);

            loop_rate.sleep();
            success = move_group.plan(my_plan);
            if (success == moveit::planning_interface::MoveItErrorCode::SUCCESS)
                move_group.execute(my_plan);

            // Open the gripper and detach the can
            gripper_goal.command.position = 0.115;
            gripper_action_client_->sendGoal(gripper_goal);
            move_group.detachObject("Can1");
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
        // std::cout << "Waiting the user to move close."<< std::endl;
        loop_rate.sleep();
    }

    spinner.stop();
    ros::shutdown();
}

void FetchViconDemo::waitForUserConfirmation(const std::string& next_task)
{
    // std::cout << "Robot is about to ";
    if (isatty(STDOUT_FILENO))
    {
        // std::cout << "\033[1;32m<" << next_task << ">\033[0m. ";
    }
    else
    {
        // std::cout << "<" << next_task << ">. ";
    }
    // std::cout << "Press enter when ready."<< std::endl;
    std::cin.ignore();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fetch_vicon_demo");

    FetchViconDemo demo;
    demo.run();

    return 0;
}
