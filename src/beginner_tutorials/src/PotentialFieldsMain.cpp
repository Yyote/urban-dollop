// PotentialFields algorithm for lidar in PointCloud  !!2D!!
// 
// Listens to topic /transPC
// Publishes to topic /potential_field_result
// 
// frame: laser
// 
#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include "beginner_tutorials/VelocitiesPC.h"
#include "geometry_msgs/Quaternion.h"
#include "tf2/LinearMath/Quaternion.h"
#include "visualization_msgs/Marker.h"
#include <cmath>

// PRESETUP 
bool publish_rviz_vizualization = 1;
int sessionID = 0;


// SUB catchPC creation
ros::Subscriber catchPC_sub;

ros::Publisher vis_pub;

struct EulerAngles {
    float roll;
    float pitch;
    float yaw;
};


void got_scanCallback(const sensor_msgs::PointCloud::ConstPtr& catchedCloud)
{
    ROS_INFO_STREAM("Size of is --->" << catchedCloud->points.capacity() << std::endl);

    tf2::Quaternion q;
    EulerAngles angles;

        angles.roll = 0;
        angles.pitch = 0;
        angles.yaw = 0;

    visualization_msgs::Marker marker; // RVIZ marker init

    // CODEINFO
    // Calculate average x and y and make a vector of retraction from them
    // PROBLEM 1
    // Algorithms calculates retraction for all points - this makes problems when some points get out of sight
    // PROBLEM 2
    // For an average vector change in 1-20 points is very little. That is a problem when the robot gets close to objects as speed vector is changing too little
    double xsum, ysum, sumcount;
    xsum = 0;
    ysum = 0;
    sumcount = 0;
    sensor_msgs::PointCloud vel_cloud;
    vel_cloud.points.resize(catchedCloud->points.capacity());
    float finvec_x = 0;
    float finvec_y = 0;
    //DEBUG
    ROS_INFO_STREAM("catchedCloud capacity is ---> " << catchedCloud->points.capacity());
    for(int i = 0; i < catchedCloud->points.capacity(); i++) //SOLVED getting out of range ---> you should be more careful when working with cycle's iterator incrementation
    {
        if(sqrt(pow(catchedCloud->points.at(i).x, 2) + pow(catchedCloud->points.at(i).y, 2)) >= 1.5)
        {
            vel_cloud.points.at(i).x = 0;
            vel_cloud.points.at(i).y = 0;
        }

        else 
        {
            vel_cloud.points.at(i).x = 0.01 / pow(catchedCloud->points.at(i).x, 1);
            vel_cloud.points.at(i).y = 0.01 / pow(catchedCloud->points.at(i).y, 1);
            
            if(vel_cloud.points.at(i).x > 3) 
            {
                vel_cloud.points.at(i).x == 3;
            }

            else if (vel_cloud.points.at(i).y > 3) 
            {
                vel_cloud.points.at(i).y == 3;
            }
            sumcount += 1;
        }
        finvec_x += vel_cloud.points.at(i).x;
        finvec_y += vel_cloud.points.at(i).y;
    //         if(sqrt(pow(catchedCloud->points.at(i).x, 2) + pow(catchedCloud->points.at(i).y, 2)) >= 1.5)
    //         {
    //             xsum += 0;
    //             ysum += 0;
    //         }
    //         else 
    //         {
    //         xsum += catchedCloud->points.at(i).x;
    //         ysum += catchedCloud->points.at(i).y;
    //         sumcount = 1;
    //         }

    //     ROS_INFO_STREAM(std::endl << "_________________________________" << std::endl << "_________________________________" << std::endl << "FUNCTION NAME: average coordinates calculation" << std::endl << "VARIABLES: list of vars" << "\ncloud capacity, xsum, ysum, vector length, i, sumcount:" << std::endl << catchedCloud->points.capacity() << " "  << xsum << " " << ysum << " " << sqrt(pow(catchedCloud->points.at(i).x, 2) + pow(catchedCloud->points.at(i).y, 2)) << " " << i << " " << sumcount << std::endl << "_________________________________" << std::endl << "_________________________________" << std::endl);
    // }
        ROS_INFO_STREAM(std::endl << "_________________________________" << std::endl << "_________________________________" << std::endl << "FUNCTION NAME: average coordinates calculation" << std::endl << "VARIABLES: list of vars" << "\ncloud capacity, vel_cloud points x and y, vector length, i, sumcount:" << std::endl << catchedCloud->points.capacity() << " "  << vel_cloud.points.at(i).x << " " << vel_cloud.points.at(i).y << " " << sqrt(pow(vel_cloud.points.at(i).x, 2) + pow(vel_cloud.points.at(i).y, 2)) << " " << i << " " << sumcount << std::endl << "finvec_x -->" << finvec_x << std::endl << "finvec_y -->" << finvec_y << std::endl << "_________________________________" << std::endl << "_________________________________" << std::endl);

    }
    ROS_WARN_STREAM(std::endl << "_________________________________" << std::endl << "_________________________________" << std::endl << "FUNCTION NAME: result vector in variables" << std::endl << "VARIABLES: list of vars:" << std::endl << "finvec_x -->" << finvec_x << std::endl << "finvec_y -->" << finvec_y << std::endl << "vector length -->" << sqrt(pow(finvec_x , 2) + pow(finvec_y , 2)) << "_________________________________" << std::endl << "_________________________________" << std::endl);
    finvec_x = finvec_x;
    finvec_y = finvec_y;
    // xsum = xsum / sumcount;
    // ysum = ysum / sumcount;

    //IDEA make a bullshit filter for potential fields and create max passed speed constant

    // // AAA Here I dump distance data from my msg into an PC to be able to visualize published data //RETHINK the idea of visualizng point cloud as speeds
    //         PointCloud_velocity_points.points.at(i).x = velocities.velocityModule.points.at(i).x; //IDEA visualize the points' speeds as PointCloud and the final vector as marker
    //         PointCloud_velocity_points.points.at(i).y = velocities.velocityModule.points.at(i).y;

    // CODEINFO angle calculation to pass to rviz markers ---> calculate current(i) vector's angle 
    //IDEA replace angles object with angles array to store angle for each vector
    angles.yaw = atan(finvec_y / finvec_x); //IDEA claculate angle one time for the resulting vector
    q.setRPY(angles.roll, angles.pitch, angles.yaw);
    q = q.normalize();

    //*-*-*-*-*-*-*-*--*-*-*-*-*--*--*-*-*-*-*-*-*--*--*-*-*-*-*-*-*-*-*-*-*--*-*-*-*
    //------------------------------------------------------------------------------
    // VISUALIZATION //RVIZ visualiztion code // TODO make it a func
    //------------------------------------------------------------------------------
    // LOG NEW removed marker initalization to change to markerarray
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = "laser"; //FRAME
    marker.ns = "speeds_namespace";
    marker.id = sessionID;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z(); //SOLVED can't make the vector turn ---> check function, everything should work if done properly  //TODO: put a ROS_INFO and a ROS_WARN on marker.scale.x
    marker.pose.orientation.w = q.w();
    marker.scale.x = sqrt(pow(finvec_x, 2) + pow(finvec_y, 2)); //TODO: change calculations for vector average //IDEA calculate average x and y and use them to calculate speed
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    //*-*-*-*-*-*-*-*--*-*-*-*-*--*--*-*-*-*-*-*-*--*--*-*-*-*-*-*-*-*-*-*-*--*-*-*-*


    if(publish_rviz_vizualization == 1)
    {
        vis_pub.publish(marker); // PUB //RVIZ publish
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "potential_fields");

// NODEHANDLE
ros::NodeHandle cs;
ros::NodeHandle vlcts; //TODO check for unused nhs
ros::NodeHandle rvizor; // mrviz markers nh
ros::NodeHandle vlcts_in_PC;

    //PUB adverts
    vis_pub = rvizor.advertise<visualization_msgs::Marker>("/potential_field_result", 10);
    catchPC_sub = cs.subscribe("/transPC", 1000, got_scanCallback); // SUB catchPC sub
    ros::spin();

    return 0;
}

