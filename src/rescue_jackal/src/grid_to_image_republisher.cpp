
#include <ros/ros.h>

// #include "grid_to_image_republisher.h"

#include <image_transport/image_transport.h>
#include <opencv2/imgcodecs.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>


class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //create publisher
    pub_ = n_.advertise<sensor_msgs::Image>("/grid_map_image", 1);

    //create subscriber
    sub_ = n_.subscribe("/elevation_mapping/elevation_map", 1, &SubscribeAndPublish::callback, this);
  }

  void callback(const grid_map_msgs::GridMap& msg)
  {
    grid_map::GridMap map;
    cv_bridge::CvImage image;
    grid_map::GridMapRosConverter::fromMessage(msg, map, {"elevation"});
    grid_map::GridMapRosConverter::toCvImage(map,"elevation", sensor_msgs::image_encodings::MONO8, image);
    //republish image
    pub_.publish(image);
    //ROS_INFO("Republish images");
  }

private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber sub_;

};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "grid_to_image_republisher");

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject;

  ros::spin();

  return 0;
}

// void gridMapCallback(const grid_map_msgs::GridMap& msg)
// {
//   grid_map::GridMap map;
//   cv_bridge::CvImage image;
//   grid_map::GridMapRosConverter::fromMessage(msg, map, {"elevation"});
//   grid_map::GridMapRosConverter::toCvImage(map,"elevation", sensor_msgs::image_encodings::MONO8, image);
//   ros::Publisher pub = n.advertise<sensor_msgs::image_encodings::MONO8>("grid_map_image", 1, true);
//   pub.publish(mapMessage);
//   ROS_INFO("Republish images");
// }

// int main(int argc, char **argv)
// {

//   ros::init(argc, argv, "grid_to_image_republisher");

//   ros::NodeHandle n;

//   ros::Subscriber sub = n.subscribe("/elevation_mapping/elevation_map", 1, gridMapCallback);

//   ros::spin();

//   return 0;
// }
