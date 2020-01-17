#include "bits_hardware.h"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "bits_base_node");

  ros::AsyncSpinner spinner(4);

  ros::NodeHandle nh, private_nh("~");
  std::string node_name = ros::this_node::getName();

  BitsHardware bits;
  bits.Initialize(node_name, nh, private_nh);

  spinner.start();
  ros::waitForShutdown();
  return 0;
}