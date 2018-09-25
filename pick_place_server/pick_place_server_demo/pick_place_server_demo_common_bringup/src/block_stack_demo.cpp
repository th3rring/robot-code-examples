#include <ros/ros.h>
#include <pick_place_client.h>

using namespace pick_place_server_client;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "block_stack_demo");

    //Create a new action server client for the pick place server
    ActionClient client("pickplace", true);

    //Continue to attempt to pick and place blocks forever
    while (ros::ok())
    {
	//Make sure the server is ready for us
        client.waitForServer();

	//Specifying two waypoints.
        pickPlacePos(&client, "block_white_1", "Block Top Grasp", "base_link", {{0.6, 0, 0.4},{0.6, 0, 0.3}},
			{{0.5, 0.5, -0.5, 0.5},{0.5, 0.5, -0.5, 0.5}});

	//Specifying a full trajectory
        pickPlaceTraj(&client, "block_black_1", "Block Top Grasp",
                      "vicon_object_server/block_white_1", "Block Top Place");

	//Both of these calls follow specific waypoints of a trajectory
        pickPlaceTrajWaypoints(&client, "block_black_1", "Block Top Grasp", "base_link",
                               "Block Top Remove", {0, 1});

        pickPlaceTrajWaypoints(&client, "block_white_1", "Block Top Grasp", "base_link",
                               "Block Top Remove", {0, 2});
    }
    return 0;
}
