#include <actionlib/client/simple_action_client.h>
#include <pick_place_server_server/PickPlaceAction.h>
#include <ros/ros.h>

namespace pick_place_server_client
{

typedef actionlib::SimpleActionClient<pick_place_server_server::PickPlaceAction> ActionClient;

void pickPlaceTraj(ActionClient* client, std::string pick, std::string pick_traj_name, std::string place_frame, std::string place_traj_name);

void pickPlaceTrajWaypoints(ActionClient* client, std::string pick, std::string pick_traj_name, std::string place_frame, std::string place_traj_name, std::vector<uint16_t> place_traj_waypoints);

void pickPlacePos(ActionClient* client, std::string pick, std::string pick_traj_name, std::string place_frame, std::vector< std::vector<float>> place_positions, std::vector< std::vector<float>> place_orientations);
}
