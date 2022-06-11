// Lidar point velocity calculator for RPLidar
// 
// Listens to topic /limited_scan
// Publishes to topic /point_velocities, /point_velocities_in_LaserScan
// 
// LaserScan frame id laser
// 
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "beginner_tutorials/VelocitiesScan.h"
#include <math.h>

beginner_tutorials::VelocitiesScan velocities; //velocitizer
sensor_msgs::LaserScan LaserScan_velocity_points;
ros::Publisher velocities_pub;
ros::Publisher velocities_in_LaserScan_pub;


void got_scanCallback(const sensor_msgs::LaserScan::ConstPtr& catchedScan)
{

    //-*-*-*-**-*--*-*-*-*-*-*-**--*-*-*-*-*-*-*-*-**-*--*-*-*-*-*

    LaserScan_velocity_points.header.seq = catchedScan->header.seq;
    LaserScan_velocity_points.header.stamp = ros::Time::now();
    LaserScan_velocity_points.header.frame_id = "laser";
    LaserScan_velocity_points.angle_min = catchedScan->angle_min;// ИНИЦИАЛИЗАЦИЯ ПОЛЕЙ СООБЩЕНИЯ
    LaserScan_velocity_points.angle_max = catchedScan->angle_max;// ИНИЦИАЛИЗАЦИЯ ПОЛЕЙ СООБЩЕНИЯ
    LaserScan_velocity_points.angle_increment = catchedScan->angle_increment;// ИНИЦИАЛИЗАЦИЯ ПОЛЕЙ СООБЩЕНИЯ
    LaserScan_velocity_points.time_increment = catchedScan->time_increment;// ИНИЦИАЛИЗАЦИЯ ПОЛЕЙ СООБЩЕНИЯ
    LaserScan_velocity_points.scan_time = catchedScan->scan_time;// ИНИЦИАЛИЗАЦИЯ ПОЛЕЙ СООБЩЕНИЯ
    LaserScan_velocity_points.range_min = catchedScan->range_min;// ИНИЦИАЛИЗАЦИЯ ПОЛЕЙ СООБЩЕНИЯ
    LaserScan_velocity_points.range_max = catchedScan->range_max;// ИНИЦИАЛИЗАЦИЯ ПОЛЕЙ СООБЩЕНИЯ

    //*-*-*--**-**--**-*-*--*-*-*-**-*-*--*-*-*-**-*--**-*-*--*-*-

    ROS_INFO_STREAM("Got scan! Comparing..." << std::endl);


    for(int i = 0; i < 359; i++)
    {
        velocities.previousScan3.at(i) = velocities.previousScan2.at(i);
        velocities.previousScan2.at(i) = velocities.previousScan.at(i);
        velocities.previousScan.at(i) = velocities.currentScan.at(i);
        if(catchedScan->ranges.at(i) != 0.0)
        {
            velocities.currentScan.at(i) = catchedScan->ranges.at(i);
        }
        else
        {
            if(velocities.previousScan.at(i != 0))
            { 
                velocities.currentScan.at(i) = velocities.previousScan.at(i);
            }
            else
            {
                if(velocities.previousScan.at(i != 0))
                    { 
                        velocities.currentScan.at(i) = velocities.previousScan2.at(i);
                    }
                else
                    {
                        if(velocities.previousScan.at(i != 0))
                        { 
                            velocities.currentScan.at(i) = velocities.previousScan3.at(i);
                        }
                    else
                        {
                            velocities.currentScan.at(i) = velocities.previousScan.at(i);
                        }
                    }
            }
        }
        velocities.velocityModule.at(i) = ((velocities.currentScan.at(i)-velocities.previousScan.at(i)) + (velocities.currentScan.at(i)-velocities.previousScan2.at(i)) + (velocities.currentScan.at(i) - velocities.previousScan3.at(i))) / (3 * /*0.1*/ catchedScan->scan_time);
        LaserScan_velocity_points.ranges.at(i) = velocities.velocityModule.at(i);
 
        velocities_pub.publish(velocities);
        velocities_in_LaserScan_pub.publish(LaserScan_velocity_points);
        ROS_INFO_STREAM("Published velocities successfully!" << std::endl);
    }
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "scan_velocitizer");

    ros::NodeHandle cs;
    ros::NodeHandle vlcts;
    ros::NodeHandle rvizor;
    ros::NodeHandle vlcts_in_LS;

    ros::Subscriber catchScans_sub;
    velocities_pub = vlcts.advertise<beginner_tutorials::VelocitiesScan>("point_velocities", 100);
    velocities_in_LaserScan_pub = vlcts_in_LS.advertise<sensor_msgs::LaserScan>("point_velocities_in_LaserScan", 100);

    velocities.velocityModule.resize(360);
    velocities.currentScan.resize(360);
    velocities.previousScan.resize(360);
    velocities.previousScan2.resize(360);
    velocities.previousScan3.resize(360);
    LaserScan_velocity_points.ranges.resize(360);

    // ros::Rate loop_rate(10); // NEW

// while (ros::ok())
// { 


    catchScans_sub = cs.subscribe("limited_scan", 1000, got_scanCallback);
    
    ros::spin();
    // ros::spinOnce();// NEW
    // loop_rate.sleep();// NEW

// }
    return 0;
}

