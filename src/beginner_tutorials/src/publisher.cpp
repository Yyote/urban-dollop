#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  ros::Publisher turtleMove_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000); // /turtle1

  ros::Rate loop_rate(0.5);

  int count = 0;

  while (ros::ok())
  {
    geometry_msgs::Twist velocity;
    if (count%3 == 0){
      velocity.linear.x = 1.8;
      velocity.angular.z = 1.8;
    }
    else if (count % 3 == 1) {
      velocity.linear.x = 1.8;
      velocity.angular.z = -1.8;
    }
    else {
      velocity.angular.z = -1.8;

    }

    turtleMove_pub.publish(velocity);
    ros::spinOnce();
    loop_rate.sleep();
    count++;
  }
  return 0;
}
