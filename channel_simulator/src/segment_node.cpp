#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <gazebo_msgs/GetModelState.h>
#include <channel_simulator/channel_simulator.h>
#include <channel_simulator/PathlossPair.h>
#include <channel_simulator/PathlossPairArray.h>
#include <string.h>
#include <math.h>
#include <vector>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "communication_pathloss_node");
  ros::NodeHandle nh, pnh("~");

  channel_simulator::ChannelSimulator channel_sim(nh);

  ros::Publisher pathloss_pub = nh.advertise<channel_simulator::PathlossPairArray>("pathloss", 10);
  ros::Subscriber map_sub = nh.subscribe("octomap", 1, &channel_simulator::ChannelSimulator::mapCB,
                                         &channel_sim);

	ros::ServiceClient sc;
  sc = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

  XmlRpc::XmlRpcValue gazebo_models_list;
  if (!nh.getParam("/gazebo_models", gazebo_models_list)) {
    ROS_FATAL("failed to fetch required param \"/gazebo_models\" from server");
    exit(EXIT_FAILURE);
  }
  ROS_ASSERT(gazebo_models_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

  std::vector<std::string> gazebo_models;
  for (int i = 0; i < gazebo_models_list.size(); ++i) {
    ROS_ASSERT(gazebo_models_list[i].getType() == XmlRpc::XmlRpcValue::TypeString);
    gazebo_models.push_back(static_cast<std::string>(gazebo_models_list[i]));
  }
  int number_of_agents = gazebo_models.size();

  // compute pair permutations
  std::vector<channel_simulator::PathlossPair> perms;
  for (int i = 1; i < number_of_agents; ++i) {
    for (int j = i+1; j <= number_of_agents; ++j) {
      channel_simulator::PathlossPair pair;
      pair.id1 = i;
      pair.id2 = j;
      perms.push_back(pair);
    }
  }

  // wait for model service to become available
  sc.waitForExistence();

  ros::Rate rate(10);
  while (ros::ok()) {
    rate.sleep();
    ros::spinOnce();

    // fetch model state from gazebo
    std::vector<geometry_msgs::Point> node_pos;
    for (auto& model : gazebo_models) {
      gazebo_msgs::GetModelState model_state;
      model_state.request.model_name =  model;
      if (!sc.call(model_state)) {
        std::string service_name = sc.getService();
        ROS_WARN("[path_segments] failed to fetch state of %s from %s", model_state.request.model_name.c_str(), service_name.c_str());
        break;
      }
      node_pos.push_back(model_state.response.pose.position);
    }

    // if one of the get_model_state service calls failed then start loop over
    if (node_pos.size() != number_of_agents)
      continue;

    channel_simulator::PathlossPairArray paths;
    // compute path segments for each permutation of robots
    for (channel_simulator::PathlossPair perm : perms) {
      perm.path_points = channel_sim.computeSegments(node_pos[perm.id1-1], node_pos[perm.id2-1]);
      paths.path_array.push_back(perm);
    }

    pathloss_pub.publish(paths);
  }

  return 0;
}
