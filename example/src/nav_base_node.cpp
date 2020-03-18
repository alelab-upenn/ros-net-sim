#include <example/nav_base.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "nav_base_node");
  ros::NodeHandle nh, pnh("~");

  ros::AsyncSpinner spinner(4);
  spinner.start();

  example::NavBase nav(ros::this_node::getName(), nh, pnh);

  ros::waitForShutdown();

  return 0;
}
