// Lidar point velocity calculator for RPLidar in PointCloud
// 
// Listens to topic /transPC
// Publishes to topic /point_velocities_PC_custmoMSG, /point_velocities_in_PC
// 
// 
// #include <vector>
#include "ros/ros.h"
//#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "beginner_tutorials/VelocitiesPC.h"
#include "geometry_msgs/Quaternion.h"
#include "tf2/LinearMath/Quaternion.h"
#include "visualization_msgs/MarkerArray.h" //LOG NEW changed Marker.h to MarkerArray.h
#include <cmath>

    bool publish_rviz_vizualization = 1;


//PUB inits
beginner_tutorials::VelocitiesPC velocities; //velocitizer
sensor_msgs::PointCloud PointCloud_velocity_points;
ros::Publisher velocities_pub;
ros::Publisher velocities_in_PC_pub;
ros::Publisher vis_pub;

struct EulerAngles {
    float roll;
    float pitch;
    float yaw;
};


// TODO: class implementation
// class PointSpeedCalculator
// {
//     private: 
//     float scan_time_previous = 0;
//     float scan_time = 0;
//     float range_limit = 0;
//     int bullshit_counter = 0;

//     public:
//     sensor_msgs::PointCloud PointCloud_velocity_points;
//     beginner_tutorials::VelocitiesPC velocities; //velocitizer
//     PointSpeedCalculator(/*const sensor_msgs::PointCloud::ConstPtr& catchedCloud*/)
//     {
//     }
//     void calcSpeeds(const sensor_msgs::PointCloud::ConstPtr& catchedCloud, PointCloud_velocity_points,)
//     {

//     }
//     int calcPolarityPC();
// };

// AAA Calculates if vector is positive or negative to set it's orientation in rviz marker
int radVec_quarter_calc(float x, float y)
{
    int quar = -1;
    if (x > 0)
    {
        if (y > 0)
        {   
            quar = 1;
        }
        else if (y < 0)
        {
            quar = 4;
        }
        else quar = -1;
    }
    else if (x < 0)
    {
        if (y > 0)
        {   
            quar = 2;
        }
        else if (y < 0)
        {
            quar = 3;
        }
        else quar = -1;
    }
    else quar = -1;
    //DEBUG
    if (quar == -1) ROS_ERROR_STREAM("COULD NOT CALCULATE QUARTER, QUAR IS ---> " << quar << std::endl);
    ROS_INFO_STREAM(std::endl << "-----------------------------------------------------" << std::endl << "radVec_quarter_calc DEBUG" << std::endl << "-----------------------------------------------------" << std::endl << "x, y, quar ---> " << x << ", " << y << ", " << quar << std::endl << std::endl << std::endl);
    return quar;
}

int polarityVariant_calc(int q2, int q1) //q1 is current q2 is prev //NO
{
    // // AAA not allow q2 to be
    // if (q2 > q1)
    // {
    //     int qtemp;
    //     qtemp = q1;
    //     q1 = q2;
    //     q2 = qtemp;
    // }

    int polvar = -1;
    if (q1 == 1)
    {
        if (q2 == 1)
        {
            polvar = 1;
        }
        else if (q2 == 3)
        {
            polvar = 51;
        }
    }
    if (q1 == 2)
    {
        if (q2 == 2)
        {
            polvar = 2;
        }
        else if (q2 == 4)
        {
            polvar = 61;
        }
    }
    if (q1 == 3)
    {
        if (q2 == 3)
        {
            polvar = 3;
        }
        else if (q2 == 1)
        {
            polvar = 52;
        }
    }
    else if (q1 == 4)
    {
        if (q2 == 4)
        {
            polvar = 4;
        }
        else if (q2 == 2)
        {
            polvar = 62;
        }
    }
    //DEBUG
    ROS_INFO_STREAM(std::endl << "-----------------------------------------------------" << std::endl << "polarityVariant_calc DEBUG" << std::endl << "-----------------------------------------------------" << std::endl << "q1, q2, polvar ---> " << q1 << ", " << q2 << ", " << polvar << std::endl << std::endl << std::endl);
    return polvar;
}

int calcPolarityPC(beginner_tutorials::VelocitiesPC cloud, int i) //LOG trying to pass the cloud pointer object into callback ---> removed pointer
{
    // AAA Takes current and previous radius vectors' coordinates and finds the quarter in which the vectors and the delta vector is situated
    // AAA Then compares and sets polarity
    int polarity;
    int q2, q1, polvar; //LOG NEW dx dy previously were declared as ints (!!!) XD
    double x1, x2, y1, y2, dx, dy; // create coordinate variables for 2 dots where the first dot is current(x1;y1) and the second dot is previous(x2;y2)
    x2 = cloud.currentScan.points.at(i).x;
    y2 = cloud.currentScan.points.at(i).y;
    x1 = /*(*/cloud.previousScan.points.at(i).x/* + cloud.previousScan2.points.at(i).x + cloud.previousScan3.points.at(i).x) / 3*/;
    y1 = /*(*/cloud.previousScan.points.at(i).y/* + cloud.previousScan2.points.at(i).y + cloud.previousScan3.points.at(i).y) / 3*/;
    dx = x2 - x1; //TODO: check for r2 - r1
    dy = y2 - y1;
    q1 = radVec_quarter_calc(x1, y1);
    q2 = radVec_quarter_calc(x2, y2);
    polvar = polarityVariant_calc(q2, q1);
    //TODO: integrate math formulae for different variants of polarity
    if  (polvar == 1)
    {
        if (dx < 0 && dy < 0) polarity = -1;
        else if (dx > 0 && dy > 0) polarity = 1;
        else polarity = 0;
    }
    else if (polvar == 2)
    {
        if (dx < 0 && dy > 0) polarity = 1;
        else if (dx > 0 && dy < 0) polarity = -1;
        else polarity = 0;
    }
    else if (polvar == 3)
    {
        if (dx < 0 && dy < 0) polarity = 1;
        else if (dx > 0 && dy > 0) polarity = -1;
        else polarity = 0;
    }
    else if (polvar == 4)
    {
        if (dx < 0 && dy > 0) polarity = -1;
        else if (dx > 0 && dy < 0) polarity = 1;
        else polarity = 0;
    }
    else if (polvar == 51)
    {
        polarity = -1;
    }
    else if (polvar == 52)
    {
        polarity = 1;
    }
    else if (polvar == 61)
    {
        polarity = -1;
    }
    else if (polvar == 62)
    {
        polarity = 1;
    }
    //DEBUG
    else ROS_ERROR_STREAM("POLVAR IS NaN ---> " << polvar << std::endl);
    ROS_INFO_STREAM(std::endl << "-----------------------------------------------------" << std::endl << "calcPolarityPC DEBUG" << std::endl << "-----------------------------------------------------" << std::endl << "x2, y2, x1, y1, dx, dy ---> " << x2 << ", " << y2 << ", " << x1 << ", " << y1 << ", " << dx << ", " << dy << std::endl << "q2, q1, polvar, polarity ---> " << q2 << ", " << q1 << ", " << polvar << ", " << polarity << std::endl << std::endl << std::endl);
    
    return polarity;
}

void got_scanCallback(const sensor_msgs::PointCloud::ConstPtr& catchedCloud)
{
// AAA copy the data except points from catched LS to a local one to publish a fully documented LS
    //-*-*-*-**-*--*-*-*-*-*-*-**--*-*-*-*-*-*-*-*-**-*--*-*-*-*-*
// AAA Header fileds initialization
    PointCloud_velocity_points.header.seq = catchedCloud->header.seq;
    PointCloud_velocity_points.header.stamp = ros::Time::now();
    PointCloud_velocity_points.header.frame_id = "laser";

    //*-*-*--**-**--**-*-*--*-*-*-**-*-*--*-*-*-**-*--**-*-*--*-*- 

    ROS_INFO_STREAM("Got scan! Comparing..." << std::endl);

    //DEBUG
    ROS_INFO_STREAM("Size of is --->" << velocities.velocityModule.points.capacity() << std::endl);
    ROS_INFO_STREAM("Size of is --->" << velocities.currentScan.points.capacity() << std::endl);
    ROS_INFO_STREAM("Size of is --->" << velocities.previousScan.points.capacity() << std::endl);
    ROS_INFO_STREAM("Size of is --->" << velocities.previousScan2.points.capacity() << std::endl);
    ROS_INFO_STREAM("Size of is --->" << velocities.previousScan3.points.capacity() << std::endl);
    ROS_INFO_STREAM("Size of is --->" << catchedCloud->points.capacity() << std::endl);


// AAA angle geometry initialization
// quaternion vector for storing angle data of the points
    std::vector<tf2::Quaternion> q;
    EulerAngles angles;
    angles.roll = 0;
    angles.pitch = 0;
    angles.yaw = 0;
    q.resize(catchedCloud->points.capacity());

// AAA scan_time_previous is initialized to not make an error on first run;
    float scan_time_previous = 0;
    float scan_time;
    float range_limit = 0.0;
    int bullshit_counter = 0; // variable to count the number of meaningless speeds, that appear from crossing the min distance of the lidar view

    scan_time_previous = scan_time;
    scan_time = catchedCloud->points.at(69).z;
// AAA cloud.points.at(69).z is dedicated to pass scan_time;

// AAA create a buffer for my LS to exclude zeros (infs should be instead of zeros) from my LS and try to make data stable
// AAA                   AKA STABILIZER!@!
// AAA To buffer data I use a custom msg
// AAA TRANSFER CURRENT CLOUD INTO PREVIOUS AND MOVE PREVIOUS CLOUDS FURTHER
    visualization_msgs::MarkerArray marker;
    marker.markers.resize(catchedCloud->points.capacity()/5);

    for(int i = 0; i < catchedCloud->points.capacity(); i++)
    {
        velocities.previousScan3.points.at(i).x = velocities.previousScan2.points.at(i).x;
        velocities.previousScan3.points.at(i).y = velocities.previousScan2.points.at(i).y;

        velocities.previousScan2.points.at(i).x = velocities.previousScan.points.at(i).x;
        velocities.previousScan2.points.at(i).y = velocities.previousScan.points.at(i).y;

        velocities.previousScan.points.at(i).x = velocities.currentScan.points.at(i).x;
        velocities.previousScan.points.at(i).y = velocities.currentScan.points.at(i).y;
// AAA GET NEW CLOUD INTO currentScan
        if(sqrt(pow(catchedCloud->points.at(i).x, 2) + pow(catchedCloud->points.at(i).y, 2)) != range_limit)
        {
            velocities.currentScan.points.at(i).x = catchedCloud->points.at(i).x;
            velocities.currentScan.points.at(i).y = catchedCloud->points.at(i).y;
        }
        else
        {
            if(sqrt(pow(velocities.previousScan.points.at(i).x, 2) + pow(velocities.previousScan.points.at(i).y, 2)) != range_limit)
            { 
                velocities.currentScan.points.at(i).x = velocities.previousScan.points.at(i).x;
                velocities.currentScan.points.at(i).y = velocities.previousScan.points.at(i).y;
            }
            else
            {
                if(sqrt(pow(velocities.previousScan2.points.at(i).x, 2) + pow(velocities.previousScan2.points.at(i).y, 2)) != range_limit)
                    { 
                        velocities.currentScan.points.at(i).x = velocities.previousScan2.points.at(i).x;
                        velocities.currentScan.points.at(i).y = velocities.previousScan2.points.at(i).y;
                    }
                else
                    {
                        if(sqrt(pow(velocities.previousScan3.points.at(i).x, 2) + pow(velocities.previousScan3.points.at(i).y, 2)) != range_limit)
                        { 
                            velocities.currentScan.points.at(i).x = velocities.previousScan3.points.at(i).x;
                            velocities.currentScan.points.at(i).y = velocities.previousScan3.points.at(i).y;
                        }
                        else 
                        {
                            velocities.currentScan.points.at(i).x = catchedCloud->points.at(i).x;
                            velocities.currentScan.points.at(i).y = catchedCloud->points.at(i).y;
                        }//;
                    }
            }
        }
        //DEBUG
    ROS_INFO_STREAM("coords x --->" << velocities.currentScan.points.at(i).x << std::endl << "coords y ---> " << velocities.currentScan.points.at(i).y << std::endl);


    // AAA ##LS## Here I average out the distance data to make it even more stable        
        //     velocities.velocityModule.at(i) = ((velocities.currentScan.at(i)-velocities.previousScan.at(i)) + (velocities.currentScan.at(i)-velocities.previousScan2.at(i)) + (velocities.currentScan.at(i) - velocities.previousScan3.at(i))) / (3 * 0.1 /*catchedScan->scan_time*/);
    // AAA ##TEMPORARY## For PointCloud msg there is no scan time field, so I have to port it from the LS

    // AAA cloud.points.at(69).z is dedicated to pass scan_time;
    // AAA Check speeds for meaningless speeds for the task(even if you can see 10 meters per second, you can't always do something about it)
    if(sqrt(pow((velocities.currentScan.points.at(i).x - velocities.previousScan.points.at(i).x) / (scan_time + scan_time_previous), 2) + pow((velocities.currentScan.points.at(i).y - velocities.previousScan.points.at(i).y) / (scan_time + scan_time_previous), 2)) <= 5.0)
        {
            velocities.velocityModule.points.at(i).x = (velocities.currentScan.points.at(i).x - velocities.previousScan.points.at(i).x) / (scan_time + scan_time_previous); //LOG NEW: changed previous - current to current - previous
            velocities.velocityModule.points.at(i).y = (velocities.currentScan.points.at(i).y - velocities.previousScan.points.at(i).y) / (scan_time + scan_time_previous); //LOG NEW: same as previous line
        }
    else 
        {
            velocities.velocityModule.points.at(i).x = 0.0;
            velocities.velocityModule.points.at(i).y = 0.0;
        }

    // AAA ##IMPORTANT## bullshit counter counts the number of speeds exceeding meaningful limits. If BC exceeds some value, all the speeds become zeroes
    if (sqrt(pow((velocities.currentScan.points.at(i).x - velocities.previousScan.points.at(i).x) / (scan_time + scan_time_previous), 2) + pow((velocities.currentScan.points.at(i).y - velocities.previousScan.points.at(i).y) / (scan_time + scan_time_previous), 2)) > 3.5)
    {
        bullshit_counter++;
    }

    // AAA Here I dump distance data from my msg into an PC to be able to visualize published data
            PointCloud_velocity_points.points.at(i).x = velocities.velocityModule.points.at(i).x;
            PointCloud_velocity_points.points.at(i).y = velocities.velocityModule.points.at(i).y;

 // AAA angle calculation to pass to rviz markers
    angles.yaw = atan(velocities.currentScan.points.at(i).y / velocities.currentScan.points.at(i).x);
    // if (calcPolarityPC(velocities, i) == -1)
    // {
    //     angles.yaw += 3.14;
    // }
    // else angles.yaw = angles.yaw; //LOG scale vector inversion works too, the problem is in function
    q.at(i).setRPY(angles.roll, angles.pitch, angles.yaw);
    q.at(i)=q.at(i).normalize();

//DEBUG
int polarityprev, polarityprev2, polarityprev3, polaritycurr;
polarityprev3 = polarityprev2;
polarityprev2 = polarityprev;
polarityprev = polaritycurr;
polaritycurr = calcPolarityPC(velocities, i);

if(polaritycurr == polarityprev && polaritycurr == polarityprev2 && polaritycurr == polarityprev3)
{
    ROS_WARN_STREAM("POLARITY IS NOT CHANGING ----> " << polaritycurr << std::endl);
}
//DEBUG

    //*-*-*-*-*-*-*-*--*-*-*-*-*--*--*-*-*-*-*-*-*--*--*-*-*-*-*-*-*-*-*-*-*--*-*-*-*
    //------------------------------------------------------------------------------
    //RVIZ VIZUALIZATION
    //------------------------------------------------------------------------------
if(i % 5 == 0)
{
    //DEBUG 
    ROS_INFO_STREAM(std::endl << "-----------------------------------------------------" << std::endl << "rviz markers DEBUG" << std::endl << "-----------------------------------------------------" << std::endl << "polaritycurr, polarityprev, polarityprev2, polarityprev3 ---> " << polaritycurr << ", " << polarityprev << ", " << polarityprev2 << ", " << polarityprev3 << std::endl << "speed module, speed on x, speed on y ---> " << sqrt(pow(PointCloud_velocity_points.points.at(i).x, 2) + pow(PointCloud_velocity_points.points.at(i).y, 2)) << ", " << PointCloud_velocity_points.points.at(i).x << ", " << PointCloud_velocity_points.points.at(i).y << std::endl << std::endl << std::endl);
    //DEBUG
// LOG NEW removed marker initalization to change to markerarray
    marker.markers.at(i/5).header.stamp = ros::Time::now();
    marker.markers.at(i/5).header.frame_id = "laser"; // LOG NEW changed frame to laser
    marker.markers.at(i/5).ns = "speeds_namespace";
    marker.markers.at(i/5).id = i;
    marker.markers.at(i/5).type = visualization_msgs::Marker::ARROW;
    marker.markers.at(i/5).action = visualization_msgs::Marker::ADD;
    marker.markers.at(i/5).pose.position.x = catchedCloud->points.at(i).x;
    marker.markers.at(i/5).pose.position.y = catchedCloud->points.at(i).y;
    marker.markers.at(i/5).pose.position.z = catchedCloud->points.at(i).z;
    marker.markers.at(i/5).pose.orientation.x = q.at(i).x();
    marker.markers.at(i/5).pose.orientation.y = q.at(i).y();
    marker.markers.at(i/5).pose.orientation.z = q.at(i).z(); //BUG can't make the vector turn //TODO: put a ROS_INFO and a ROS_WARN on marker.scale.x //IDEA make debug console logs for every function to track their behaviour
    marker.markers.at(i/5).pose.orientation.w = q.at(i).w();
    marker.markers.at(i/5).scale.x = polaritycurr * sqrt(pow(PointCloud_velocity_points.points.at(i).x, 2) + pow(PointCloud_velocity_points.points.at(i).y, 2)); //TODO: check all calculations with delta r on correctness
    marker.markers.at(i/5).scale.y = 0.05;
    marker.markers.at(i/5).scale.z = 0.05;
    marker.markers.at(i/5).color.a = 1.0; // Don't forget to set the alpha!
    marker.markers.at(i/5).color.r = 0.0;
    marker.markers.at(i/5).color.g = 1.0;
    marker.markers.at(i/5).color.b = 0.0;
    //only if using a MESH_RESOURCE marker type:
}

//*-*-*-*-*-*-*-*--*-*-*-*-*--*--*-*-*-*-*-*-*--*--*-*-*-*-*-*-*-*-*-*-*--*-*-*-*

}
    //LOG NEW changed to marker array syntax
    if(publish_rviz_vizualization == 1)
    {
        vis_pub.publish( marker ); // PUB //RVIZ
    }



// AAA ##IMPORTANT## bullshit being checked and deleted
if(bullshit_counter >= 150) //130
{
    for(int i = 0; i < PointCloud_velocity_points.points.capacity(); i++)
    {
        PointCloud_velocity_points.points.at(i).x = 0;
        PointCloud_velocity_points.points.at(i).y = 0;
    }


}

// AAA Here I Publish data //PUB
        velocities_pub.publish(velocities);
        velocities_in_PC_pub.publish(PointCloud_velocity_points); //BUG velocities are zero and minus zero
        ROS_INFO_STREAM("Published velocities!" << std::endl);

    // }
    
}

int main(int argc, char **argv)
{

    //PRESETUP 
    bool publish_rviz_vizualization = 0;

    ros::init(argc, argv, "scan_velocitizer_PC");


//NODEHANDLE
    ros::NodeHandle cs;
    ros::NodeHandle vlcts;
    ros::NodeHandle rvizor; // mrviz markers nh
    ros::NodeHandle vlcts_in_PC;

    

    //SUB catchPC creation
    ros::Subscriber catchPC_sub;
    //PUB adverts
    velocities_pub = vlcts.advertise<beginner_tutorials::VelocitiesPC>("point_velocities_PC_custmoMSG", 100);
    velocities_in_PC_pub = vlcts_in_PC.advertise<sensor_msgs::PointCloud>("point_velocities_in_PC", 100);
    vis_pub = rvizor.advertise<visualization_msgs::MarkerArray>("rivz_markers_for_speed", 10);
    

    
// AAA Here I resize the fields of my custom msg
    velocities.velocityModule.points.resize(360);
    velocities.currentScan.points.resize(360);
    velocities.previousScan.points.resize(360);
    velocities.previousScan2.points.resize(360);
    velocities.previousScan3.points.resize(360);

// AAA Here I resize the ranges field for an LS that I'm going to publish
    PointCloud_velocity_points.points.resize(360);


    catchPC_sub = cs.subscribe("transPC", 1000, got_scanCallback); //SUB catchPC sub
    
    ros::spin();

    return 0;
}

