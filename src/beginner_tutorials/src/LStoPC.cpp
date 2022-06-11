// LaserScan to PointCLoud converter for a single RPLidar
// 
// Listens to topic /scan
// Publishes to topic /transPc
// frame_id laser //PC
// 
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
// #include <math.h>


ros::Subscriber Laser_sub;
ros::Publisher cloud_pub;

laser_geometry::LaserProjection projector_;


void chatterCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
  sensor_msgs::PointCloud cloud;

  cloud.points.resize(360);

  projector_.projectLaser(*scan_in, cloud);
  cloud.header.frame_id = "laser"; // PC //LOG NEW changed frame to laser
  cloud.header.seq = 69;
  cloud.header.stamp = ros::Time::now();

// AAA Dedicating points.at(69).z to scan_time to calculate velocities in other nodes
  cloud.points.at(69).z = scan_in->scan_time;

  cloud_pub.publish(cloud);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lidar_to_pointcloud");
  
  ros::NodeHandle n;

  Laser_sub = n.subscribe("limited_scan", 100, chatterCallback);
  cloud_pub = n.advertise<sensor_msgs::PointCloud>("transPC", 100);



  ros::spin();
  return 0;
}