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
#include "geometry_msgs/Twist.h"
#include <cmath>

// PRESETUP 
bool publish_rviz_vizualization = 0;
int sessionID = 0;
int okcheck = 1;
float maxRange = 1.2;

// SUB catchPC creation
ros::Subscriber catchPC_sub;

ros::Publisher vis_pub;
ros::Publisher speed_pub;

struct EulerAngles {
    float roll;
    float pitch;
    float yaw;
};

double C = 0.0005;

// class DecartCoords {
//     public:
//     double x;
//     double y;
//     double z;

//     private: 
// };

int shutdown()
{
    ROS_ERROR_STREAM(std::endl << std::endl << "terminated. code is " << okcheck);
    ros::shutdown();
    return okcheck;
}

void got_scanCallback(const sensor_msgs::PointCloud::ConstPtr& catchedCloud)
{
    // ROS_INFO_STREAM("Size of is --->" << catchedCloud->points.capacity() << std::endl);

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
    int sumcount = 0;
    sensor_msgs::PointCloud potfield;
    potfield.points.resize(catchedCloud->points.capacity());

    double finvec_x = 0;
    double finvec_y = 0;

    //DEBUG
    // ROS_INFO_STREAM("catchedCloud capacity is ---> " << catchedCloud->points.capacity());


    //IDEA visualise potential field via PointCloud(described in Obsidian)
    // sensor_msgs::PointCloud pfield;
    // double resolution, width;
    // width = 5;
    // resolution = 0.5;
    // int a = (int)width/resolution-1;
    // DecartCoords pArr[a][a];

    // double xmax, ymax, bufmax;
    // xmax = width/2;
    // ymax = xmax;
    // bufmax = xmax;

    // for(int line = 0; ymax != -bufmax; line++)
    // {
    //     for(int row = 0; xmax != -bufmax; row++)
    //     {
    //         pArr[line][row].x = xmax; //x is equivalent to row
    //         pArr[line][row].y = ymax; //y is equivalent to line
    //         xmax -= resolution
    //     }
    //     ymax -= resolution
    // }


    for(int i = 0; i < catchedCloud->points.capacity(); i++) //SOLVED getting out of range ---> you should be more careful when working with cycle's iterator incrementation
    {
        if(okcheck == 1)
        // double x, y;
        // x = catchedCloud->points.at(i).x;
        // y = catchedCloud->points.at(i).y;

        {
            if(sqrt(pow(catchedCloud->points.at(i).x, 2) + pow(catchedCloud->points.at(i).y, 2)) >= maxRange || sqrt(pow(catchedCloud->points.at(i).x, 2) + pow(catchedCloud->points.at(i).y, 2)) <= 0.05 || catchedCloud->points.at(i).x < 0.001 || catchedCloud->points.at(i).y < 0.001) //LOG NEW changed sqrt from 0.2 to 0.05
            {
                potfield.points.at(i).x = 0;
                potfield.points.at(i).y = 0;
            }
            

            else 
            {
                potfield.points.at(i).x = (C / (catchedCloud->points.at(i).x * catchedCloud->points.at(i).x)); //LOG NEW removed sqrt
                potfield.points.at(i).y = (C / (catchedCloud->points.at(i).y * catchedCloud->points.at(i).y));

                if(potfield.points.at(i).x > maxRange)
                {
                    potfield.points.at(i).x = maxRange;
                }

                else if (potfield.points.at(i).y > maxRange)
                {
                    potfield.points.at(i).y = maxRange;
                }

                sumcount += 1;
            }

            double bufx, bufy; //DEBUG
            bufx = finvec_x;
            bufy = finvec_y;
            finvec_x += -potfield.points.at(i).x;
            finvec_y += -potfield.points.at(i).y;           

            // DEBUG
            ROS_INFO_STREAM(std::endl << "_________________________________" << std::endl << "_________________________________" << std::endl << "FUNCTION NAME: average coordinates calculation" << std::endl << "VARIABLES: list of vars" << "\ncloud capacity, vel_cloud points x and y, vector length, i, sumcount:" << std::endl << catchedCloud->points.capacity() << " "  << potfield.points.at(i).x << " " << potfield.points.at(i).y << " " << sqrt(pow(potfield.points.at(i).x, 2) + pow(potfield.points.at(i).y, 2)) << " " << i << " " << sumcount << std::endl << "finvec_x -->" << finvec_x << std::endl << "finvec_y -->" << finvec_y << std::endl << "catchedCloud->points.at(i).x --> " <<  std::endl << catchedCloud->points.at(i).x << "catchedCloud->points.at(i).y --> " << catchedCloud->points.at(i).y << std::endl <<"_________________________________" << std::endl << "_________________________________" << std::endl);

            // DEBUG
            if (finvec_x == finvec_y && finvec_x != 0)
            {
                ROS_ERROR_STREAM("FINVECX IS EQ TO FINVECY. terminated.");
                shutdown();
            }

            if(finvec_x - bufx > 50 || finvec_y - bufy > 50) //DEBUG
            {
                ROS_ERROR_STREAM("FINVEC IS EXCEEDING LIMITS AND AVOIDS FILTRATION --> " << std::endl << "finvec_x ---> " << finvec_x << std::endl << "finvec_y ---> " << finvec_y);
                okcheck = -1;
                shutdown();
            }
        }
    }
    ROS_WARN_STREAM(std::endl << "_________________________________" << std::endl << "_________________________________" << std::endl << "FUNCTION NAME: result vector in variables" << std::endl << "VARIABLES: list of vars:" << std::endl << "finvec_x -->" << finvec_x << std::endl << "finvec_y -->" << finvec_y << std::endl << "vector length -->" << sqrt(pow(finvec_x , 2) + pow(finvec_y , 2)) <<  std::endl << "_________________________________" << std::endl << "_________________________________" << std::endl);


    //IDEA make a bullshit filter for potential fields and create max passed speed constant

    // // AAA Here I dump distance data from my msg into an PC to be able to visualize published data //RETHINK the idea of visualizng point cloud as speeds
    //         PointCloud_velocity_points.points.at(i).x = velocities.velocityModule.points.at(i).x; //IDEA visualize the points' speeds as PointCloud and the final vector as marker
    //         PointCloud_velocity_points.points.at(i).y = velocities.velocityModule.points.at(i).y;

    // CODEINFO angle calculation to pass to rviz markers ---> calculate current(i) vector's angle 
    //IDEA replace angles object with angles array to store angle for each vector
    angles.yaw = atan(finvec_y / finvec_x); //IDEA claculate angle one time for the resulting vector
    q.setRPY(angles.roll, angles.pitch, angles.yaw);
    q = q.normalize(); //BUG something is going with angle. cant say what, maybe its not a bug


    // DEBUG
    ROS_INFO_STREAM(std::endl << "_________________________________" << std::endl << "_________________________________" << std::endl 
    << "FUNCTION NAME: angle calc" << std::endl 
    << "VARIABLES: " << std::endl 
    << "angles.yaw (atan)-->" << angles.yaw << std::endl << "finvec_y/finvec_x --> " << finvec_y/finvec_x << std::endl << "finvec_y --> " << finvec_y << std::endl << "finvec_x --> " << finvec_x << std::endl << std::endl 
    << "_________________________________" << std::endl << "_________________________________" << std::endl);


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

    if(publish_rviz_vizualization == 1)
    {
        vis_pub.publish(marker); // PUB //RVIZ publish
    }
    //*-*-*-*-*-*-*-*--*-*-*-*-*--*--*-*-*-*-*-*-*--*--*-*-*-*-*-*-*-*-*-*-*--*-*-*-*


    //CODEINFO Twist init and pub
    geometry_msgs::Twist twist;
    if(abs(angles.yaw - 1.57) < 0.05)
    {
        twist.angular.z = 0;
        if(sqrt(pow(finvec_x, 2) + pow(finvec_y, 2)) < 0.05)
        {
            twist.linear.x = 0;
            twist.linear.y = 0;
        }
        else 
        {
            twist.linear.x = finvec_x;
            twist.linear.y = finvec_y;

        }
    }
    else
    {
        twist.angular.z = 0.4;
    }
    
    speed_pub.publish(twist);

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
    vis_pub = rvizor.advertise<visualization_msgs::Marker>("/potential_field_result", 1000);
    speed_pub = vlcts.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    catchPC_sub = cs.subscribe("/transPC", 1000, got_scanCallback); // SUB catchPC sub


    ros::spin();

    return 0;
}

