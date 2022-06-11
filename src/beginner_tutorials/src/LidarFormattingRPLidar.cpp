// Lidar formatting node for RPLidar
// 
// Listens to topic /scan
// Publishes to topic /limited_scan
// 
// 
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <math.h>

  ros::Subscriber Laser_sub;
  ros::Publisher velocities_pub_in_LaserScan_pub;

void velocities_pub_in_LaserScan_publish(const sensor_msgs::LaserScan  &catchedScan)
{
  sensor_msgs::LaserScan velocities_pub_in_LaserScan;

  velocities_pub_in_LaserScan.header.seq = catchedScan.header.seq;
  velocities_pub_in_LaserScan.header.stamp = ros::Time::now();
  velocities_pub_in_LaserScan.header.frame_id = "laser";
  velocities_pub_in_LaserScan.angle_min = catchedScan.angle_min;// ИНИЦИАЛИЗАЦИЯ ПОЛЕЙ СООБЩЕНИЯ
  velocities_pub_in_LaserScan.angle_max = catchedScan.angle_max;// ИНИЦИАЛИЗАЦИЯ ПОЛЕЙ СООБЩЕНИЯ
  velocities_pub_in_LaserScan.angle_increment = catchedScan.angle_increment;// ИНИЦИАЛИЗАЦИЯ ПОЛЕЙ СООБЩЕНИЯ
  velocities_pub_in_LaserScan.time_increment = catchedScan.time_increment;// ИНИЦИАЛИЗАЦИЯ ПОЛЕЙ СООБЩЕНИЯ
  velocities_pub_in_LaserScan.scan_time = catchedScan.scan_time;// ИНИЦИАЛИЗАЦИЯ ПОЛЕЙ СООБЩЕНИЯ
  velocities_pub_in_LaserScan.range_min = catchedScan.range_min;// ИНИЦИАЛИЗАЦИЯ ПОЛЕЙ СООБЩЕНИЯ
  velocities_pub_in_LaserScan.range_max = catchedScan.range_max;// ИНИЦИАЛИЗАЦИЯ ПОЛЕЙ СООБЩЕНИЯ
  velocities_pub_in_LaserScan.ranges.reserve(360);
  velocities_pub_in_LaserScan.ranges.resize(360);



  for(int i = 0; i < catchedScan.ranges.size(); i++)
  {
     ROS_INFO_STREAM("RANGE CAPACITY IS " << velocities_pub_in_LaserScan.ranges.size());
  ROS_INFO_STREAM("INPUT CAPACITY IS " << catchedScan.ranges.size());
    if(isfinite(catchedScan.ranges[i] ) == 1)
    {
     if(catchedScan.ranges[i] <= 5.0)
     {
       velocities_pub_in_LaserScan.ranges.at(i) = round(catchedScan.ranges[i]*10000)/10000; //???
     }
     else 
     {
       velocities_pub_in_LaserScan.ranges.at(i) = 5.0;
     }
    }
    else 
    {
     velocities_pub_in_LaserScan.ranges.at(i) = 5.0;
    }
  }

 ROS_INFO_STREAM("PUBLISHING velocities_pub_in_LaserScan...");
 ROS_INFO_STREAM("RANGE CAPACITY IS " << velocities_pub_in_LaserScan.ranges.capacity());
 ROS_INFO_STREAM("INPUT CAPACITY IS " << catchedScan.ranges.capacity());
 velocities_pub_in_LaserScan_pub.publish(velocities_pub_in_LaserScan);
}

void chatterCallback(const sensor_msgs::LaserScan::ConstPtr& piinfo)
{
  ROS_INFO_STREAM("Heard everything  --->" << piinfo->angle_min << " , " << piinfo->ranges[1]<< std::endl << std::endl << "Degree count is -> " << piinfo->ranges.size() << std::endl << "First degree is -> " << piinfo->ranges[0]);
  ROS_INFO_STREAM(piinfo->ranges.size());
  velocities_pub_in_LaserScan_publish(*piinfo);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lidar_formatting");
  ros::NodeHandle n;
  Laser_sub = n.subscribe("scan", 1000, chatterCallback);
  velocities_pub_in_LaserScan_pub = n.advertise<sensor_msgs::LaserScan>("limited_scan", 1000);



  ros::spin();
  return 0;
}