#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>

#include <protobuf_msgs/physics_update.pb.h>
#include <protobuf_msgs/channel_data.pb.h>

#include <boost/asio.hpp>

#include <boost/iostreams/filtering_streambuf.hpp>
#include <boost/iostreams/copy.hpp>
#include <boost/iostreams/filter/gzip.hpp>

#include <gazebo_msgs/GetModelState.h>
#include <channel_simulator/PathlossPair.h>
#include <channel_simulator/PathlossPairArray.h>


#define GCM_INFO(fmt, ...) ROS_INFO("[gazebo_custom_main] " fmt, ##__VA_ARGS__)
#define GCM_WARN(fmt, ...) ROS_WARN("[gazebo_custom_main] " fmt, ##__VA_ARGS__)
#define GCM_ERROR(fmt, ...) ROS_ERROR("[gazebo_custom_main] " fmt, ##__VA_ARGS__)
#define GCM_FATAL(fmt, ...) ROS_FATAL("[gazebo_custom_main] " fmt, ##__VA_ARGS__)
#define GCM_DEBUG(fmt, ...) ROS_DEBUG("[gazebo_custom_main] " fmt, ##__VA_ARGS__)


channel_simulator::PathlossPairArray pathloss_msg;
void pathloss_cb(const channel_simulator::PathlossPairArray::ConstPtr& msg)
{
  pathloss_msg = *msg;
}


static std::string gzip_compress(const std::string& data)
{
  std::stringstream compressed;
  std::stringstream origin(data);

  boost::iostreams::filtering_streambuf< boost::iostreams::input> in;
  in.push(boost::iostreams::gzip_compressor());
  in.push(origin);
  boost::iostreams::copy(in, compressed);

  return compressed.str();
}


static std::string gzip_decompress(const std::string& data)
{
  std::stringstream compressed(data);
  std::stringstream decompressed;

  boost::iostreams::filtering_streambuf<boost::iostreams::input> in;
  in.push(boost::iostreams::gzip_decompressor());
  in.push(compressed);
  boost::iostreams::copy(in, decompressed);

  return decompressed.str();
}


std::string receive_one_message(boost::asio::local::stream_protocol::socket &sock)
{
  // Read Preamble
  uint32_t data_preamble[4];
  size_t length = sock.receive(boost::asio::buffer(data_preamble, 4));
  uint32_t receive_length=ntohl(*data_preamble);
  // Read Message
  char data[receive_length];
  length = sock.receive(boost::asio::buffer(data, receive_length));
  std::string data_string(data,length);

  return data_string;
}


void send_one_message(boost::asio::local::stream_protocol::socket &sock, std::string str)
{
  // Send Preamble
  std::size_t response_size=str.size();
  static_cast<uint32_t>(response_size);
  uint32_t send_length=htonl(response_size);
  sock.send(boost::asio::buffer(&send_length,4));
  // Send Message
  sock.send(boost::asio::buffer(str.data(), str.size()));
}


std::string generate_channel_data(ros::NodeHandle &node_handle,
                                  gazebo::physics::WorldPtr& world,
                                  std::vector<std::string>& gazebo_models)
{
  channel_data_proto::ChannelData ChannelData_msg;

  for (auto& model : gazebo_models) {
    gazebo::physics::ModelPtr model_ptr = world->ModelByName(model);
    ignition::math::Pose3d pose = model_ptr->WorldPose();
    ignition::math::Quaterniond quat = pose.Rot();
    ignition::math::Vector3d pos = pose.Pos();

    ChannelData_msg.add_node_list(pos.X());
    ChannelData_msg.add_node_list(pos.Y());
    ChannelData_msg.add_node_list(pos.Z());
    ChannelData_msg.add_node_list(quat.W());
    ChannelData_msg.add_node_list(quat.X());
    ChannelData_msg.add_node_list(quat.Y());
    ChannelData_msg.add_node_list(quat.Z());
  }

  // compute pathloss msg
  for (const channel_simulator::PathlossPair& pair : pathloss_msg.path_array) {
    channel_data_proto::PathDetails* PathDetails_msg = ChannelData_msg.add_path_details();
    PathDetails_msg->add_ids(pair.id1);
    PathDetails_msg->add_ids(pair.id2);
    std::vector<double> segments = pair.path;
    std::vector<geometry_msgs::Point> segment_pts = pair.path_points;
    if (segment_pts.size() >= 1) {
      PathDetails_msg->set_los(false);
    } else {
      PathDetails_msg->set_los(true);
    }
    PathDetails_msg->add_num_hops(segment_pts.size());
      for(int j=0; j < segment_pts.size(); ++j) {
      PathDetails_msg->add_hop_points(segment_pts[j].x); // X
      PathDetails_msg->add_hop_points(segment_pts[j].y); // Y
      PathDetails_msg->add_hop_points(segment_pts[j].z); // Z
      PathDetails_msg->add_hop_points(0); // Loss
    }
  }

  std::string ChannelData_string;
  ChannelData_msg.SerializeToString(&ChannelData_string);

  return ChannelData_string;
}


std::string generate_response(std::string channel_data, physics_update_proto::PhysicsUpdate PhysicsUpdate_msg)
{
  PhysicsUpdate_msg.set_msg_type(physics_update_proto::PhysicsUpdate::END);
  PhysicsUpdate_msg.set_channel_data(gzip_compress(channel_data));
  std::string str_response;
  PhysicsUpdate_msg.SerializeToString(&str_response);

  return str_response;
}


int main(int argc, char **argv)
{
  /*
   * Initialize Gazebo
   * libgazebo_ros_api_plugin initializes a ROS node; in order to access ROS
   * components here in the same process, all ROS components must be created
   * after that plugin has loaded.
   */

  // convert args to vector of strings
  std::vector<std::string> argvec;
  for (int i = 0; i < argc; ++i)
    argvec.push_back(argv[i]);

  // find plugins to add
  for (auto& str : argvec) {
    if (str.find(".so") != std::string::npos) {
      gazebo::addPlugin(str);
      GCM_INFO("added plugin %s", str.c_str());
    }
  }

  // start server server
  GCM_INFO("setting up server");
  gazebo::setupServer(argvec);

  // command line args only get processed by system plugins when running gazebo
  // as a library so we need to parse any pertinent ones ourselves [see
  // docstring for gazebo.hh:setupServer]
  for (const std::string& arg : argvec) {
    if (arg.compare("--verbose") == 0) {
      GCM_INFO("setting gazebo logging to verbose");
      gazebo::printVersion();
      gazebo::common::Console::SetQuiet(false);
    }
  }

  // parse args for world file
  std::string world_file;
  for (auto& str : argvec) {
    if (str.find(".world") != std::string::npos) {
      world_file = str;
      GCM_INFO("world file arg: %s", str.c_str());
      break;
    }
  }

  // Load a world
  gazebo::physics::WorldPtr world;
  if (world_file.empty()) {
    GCM_FATAL("failed to find world file");
    exit(EXIT_FAILURE);
  } else {
    GCM_INFO("loading world file %s", world_file.c_str());
    world = gazebo::loadWorld(world_file);
  }

  // ensure world file is valid
  if (!world) {
    GCM_FATAL("failed to load world file");
    exit(EXIT_FAILURE);
  }

  //wait for libgazebo_ros_api_plugin to initialize ROS node
  ros::Rate delay(10);
  while (!ros::isInitialized()) {
    ROS_INFO_THROTTLE(1.0, "[gazebo_custom_main] waiting for libgazebo_ros_api_plugin to initialize");
    delay.sleep();
  }

  ros::NodeHandle nh("gazebo_custom_main"), pnh("~");
  ros::Subscriber pathloss_sub = nh.subscribe<channel_simulator::PathlossPairArray>("/pathloss",10, pathloss_cb);

  int simulation_step_time_ms;
  if (!pnh.getParam("simulation_step_time_ms", simulation_step_time_ms)) {
    simulation_step_time_ms = 10; // milliseconds
    GCM_WARN("unable to load \"simulation_step_time_ms\""
             "parameter, using default value of %d ms", simulation_step_time_ms);
  }

  // use the current gazebo physics properties to determine the appropriate
  // number of gazebo simulation steps to run during each sync iteration
  gazebo::physics::PhysicsEnginePtr pe = world->Physics();
  int gazebo_iterations;
  if (!pe) {
    gazebo_iterations = 1e-3*simulation_step_time_ms / 0.001; // default is 0.001
    GCM_WARN("unable to get physics engine properties, "
             "setting gazebo_iterations to %d", gazebo_iterations);
  } else {
    gazebo_iterations = 1e-3*simulation_step_time_ms / pe->GetMaxStepSize();
    GCM_INFO("current gazebo simulation timestep: %.3f s,"
             " desired simulation step time: %d ms; gazebo simulation will "
             "advance %d iterations each sync cycle", pe->GetMaxStepSize(),
             simulation_step_time_ms, gazebo_iterations);
  }

  // TODO get rid of this
  // run world until the agents have spawned

  XmlRpc::XmlRpcValue gazebo_models_list;
  if (!nh.getParam("/gazebo_models", gazebo_models_list)) {
    GCM_FATAL("failed to fetch required param \"/gazebo_models\" from server");
    exit(EXIT_FAILURE);
  }
  ROS_ASSERT(gazebo_models_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

  std::vector<std::string> gazebo_models;
  for (int i = 0; i < gazebo_models_list.size(); ++i) {
    ROS_ASSERT(gazebo_models_list[i].getType() == XmlRpc::XmlRpcValue::TypeString);
    gazebo_models.push_back(static_cast<std::string>(gazebo_models_list[i]));
  }

  for (auto& model : gazebo_models) {
    gazebo::physics::ModelPtr model_ptr = world->ModelByName(model);
    while (model_ptr == nullptr) {
      gazebo::runWorld(world, 1);
      model_ptr = world->ModelByName(model);
      ROS_INFO_THROTTLE(1.0, "[gazebo_custom_main] waiting for %s to spawn", model.c_str());
    }
    GCM_INFO("%s has spawned", model.c_str());
  }
  GCM_INFO("all models have been loaded");

  GCM_INFO("setting up UDS socket");
  //Create and connect UDS Socket
  boost::asio::io_service io_service;
  ::unlink("/tmp/phy_server_socket");
  boost::asio::local::stream_protocol::endpoint ep("/tmp/phy_server_socket");
  boost::asio::local::stream_protocol::acceptor acceptor(io_service, ep);
  boost::asio::local::stream_protocol::socket socket(io_service);
  acceptor.accept(socket);
  GCM_INFO("finished setting up UDS socket");

  /*
   * main simulation loop
   */
  GCM_INFO("entering main simulation loop");
  while (ros::ok()) {

    std::string received_data=gzip_decompress(receive_one_message(socket));
    physics_update_proto::PhysicsUpdate PhysicsUpdate_msg;
    PhysicsUpdate_msg.ParseFromString(received_data);

    // advance gazebo simulation desired number of timesteps
    gazebo::runWorld(world, gazebo_iterations);

    std::string channel_data=generate_channel_data(nh, world, gazebo_models);
    std::string response=gzip_compress(generate_response(channel_data, PhysicsUpdate_msg));
    send_one_message(socket, response);

    ros::spinOnce();
  }

  gazebo::shutdown();
}
