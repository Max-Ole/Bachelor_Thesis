#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>

using namespace grid_map; //?

bool add(beginner_tutorials::AddTwoInts::Request  &req,
         beginner_tutorials::AddTwoInts::Response &res)
{
  // Position position;
  // float& GridMap::atPosition(const std::string& layer, const Position& position)
  res.sum = req.a + req.b;
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "readMapCell_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("readMapCell", add);

  ros::spin();

  return 0;
}
