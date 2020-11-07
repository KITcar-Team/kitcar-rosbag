#include <cstdlib>
#include <string>
#include <boost/filesystem.hpp>
#include <ros/ros.h>
#include <rosbag/recorder.h>

namespace fs = boost::filesystem;
using namespace std::string_literals;

int main(int argc, char** argv) {
  ros::init(argc, argv, "record_snapshot");

  rosbag::RecorderOptions opts;

  const char* home = std::getenv("HOME");
  if (home == nullptr) {
    ROS_FATAL("HOME environment variable not set");
    return 1;
  }

  ros::NodeHandle nh("~");

  fs::path directory(
      home + "/"s + nh.param<std::string>("snapshot_path", "rosbag_snapshots") +
      "/");

  if (!fs::exists(directory) && !fs::create_directory(directory)) {
    ROS_FATAL("record_snapshot was not able to create directory for rosbags %s",
              directory.string().c_str());
    return 1;
  }

  constexpr uint32_t byte_to_mb = 1048576;
  opts.snapshot = true;
  opts.buffer_size = byte_to_mb * nh.param<int>("snapshot_size", 1024);
  opts.prefix =
      directory.string() +
      nh.param<std::string>("snapshot_file_prefix", "rosbag_snapshot");

  opts.topics = nh.param<std::vector<std::string>>(
      "topics_to_record", {"/tf", "/camera/image_raw"});

  ROS_INFO("started rosbag snapshot record\n");
  ROS_INFO(
      "publish a std_msgs/Empty on the topic /snapshot_trigger to record what "
      "just happened");
  ROS_INFO("the rosbag is then saved to ~/%s", directory.string().c_str());


  // Run the recorder
  rosbag::Recorder recorder(opts);
  const int result = recorder.run();

  return result;
}
