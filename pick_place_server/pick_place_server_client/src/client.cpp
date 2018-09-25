#include <pick_place_client.h>


void pick_place_server_client::pickPlaceTraj(ActionClient* client, std::string pick, std::string pick_traj_name, std::string place_frame, std::string place_traj_name)
{

    client->waitForServer();

    pick_place_server_server::PickPlaceGoal goal;
    goal.pick = pick;
    goal.pick_traj_name = pick_traj_name;
    goal.place_frame = place_frame;
    goal.place_traj_name = place_traj_name;

    client->sendGoal(goal);
    client->waitForResult(ros::Duration(300.0));

    if (client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
	ROS_INFO("Execution successful!");
    } else {
	ROS_INFO("Execution failed");
    }
}

void pick_place_server_client::pickPlaceTrajWaypoints(ActionClient* client, std::string pick, std::string pick_traj_name, std::string place_frame, std::string place_traj_name, std::vector<uint16_t> place_traj_waypoints)
{

    client->waitForServer();

    pick_place_server_server::PickPlaceGoal goal;
    goal.pick = pick;
    goal.pick_traj_name = pick_traj_name;
    goal.place_frame = place_frame;
    goal.place_traj_name = place_traj_name;
    goal.place_traj_waypoints = place_traj_waypoints;

    client->sendGoal(goal);
    client->waitForResult(ros::Duration(300.0));

    if (client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
	ROS_INFO("Execution successful!");
    } else {
	ROS_INFO("Execution failed");
    }
}

void pick_place_server_client::pickPlacePos(ActionClient* client, std::string pick, std::string pick_traj_name, std::string place_frame,  std::vector< std::vector<float>> place_positions, std::vector< std::vector<float>> place_orientations)
{

    client->waitForServer();

    uint8_t num_place_poses = place_positions.size();
    std::vector<float> place_positions_split;
    std::vector<float> place_orientations_split;

    for(int i = 0; i < num_place_poses; i++)
    {
    place_positions_split.push_back(place_positions[i][0]);
    place_positions_split.push_back(place_positions[i][1]);
    place_positions_split.push_back(place_positions[i][2]);
    
    place_orientations_split.push_back(place_orientations[i][0]);
    place_orientations_split.push_back(place_orientations[i][1]);
    place_orientations_split.push_back(place_orientations[i][2]);
    place_orientations_split.push_back(place_orientations[i][3]);
    }



    pick_place_server_server::PickPlaceGoal goal;
    goal.pick = pick;
    goal.pick_traj_name = pick_traj_name;
    goal.place_frame = place_frame;
    goal.place_positions = place_positions_split;
    goal.place_orientations = place_orientations_split;
    goal.num_place_poses = num_place_poses;

    client->sendGoal(goal);
    client->waitForResult(ros::Duration(300.0));

    if (client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
	ROS_INFO("Execution successful!");
    } else {
	ROS_INFO("Execution failed");
    }
}

