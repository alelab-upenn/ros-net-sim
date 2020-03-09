#include <channel_simulator/channel_simulator.h>
#include <octomap_msgs/conversions.h>
#include <string.h>


int main(int argc, char** argv)
{
  std::string file_name(argv[1]);

  octomap::OcTree* tree = new octomap::OcTree(0.05);
  bool success = tree->readBinary(file_name);
  if (!success) {
    ROS_INFO("failed to load octomap from file %s", file_name.c_str());
    delete tree;
    return -1;
  }

  octomap_msgs::Octomap msg;
  if (!octomap_msgs::binaryMapToMsg(*tree, msg)) {
    ROS_INFO("failed to create ROS msg from octree");
    delete tree;
    return -1;
  }
  octomap_msgs::Octomap::ConstPtr msg_ptr(new octomap_msgs::Octomap(msg));

  channel_simulator::ChannelSimulator sim;
  sim.mapCB(msg_ptr);

  geometry_msgs::Point pt1, pt2;
  pt1.x = 3.0;
  pt1.y = 4.0;
  pt1.z = 2.6;
  pt2.x = -3.0;
  pt2.y = -4.0;
  pt2.z = 2.9;
  printf("pt1 = {%.3f, %.3f, %.3f}\n", pt1.x, pt1.y, pt1.z);
  printf("pt2 = {%.3f, %.3f, %.3f}\n", pt2.x, pt2.y, pt2.z);

  std::vector<geometry_msgs::Point> pts = sim.computeSegments(pt1, pt2);
  for (auto& pt : pts) {
    printf("{%.3f, %.3f, %.3f}\n", pt.x, pt.y, pt.z);
  }

  delete tree;
  return 0;
}
