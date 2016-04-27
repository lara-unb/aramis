#include "p2os_msgs/SonarArray.h"
#include "geometry_msgs/Point32.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/ChannelFloat32.h"
#include "tf/tf.h"
#include <boost/lexical_cast.hpp>
#include <cmath>
#include <string>
#include <vector>
#include <cstdlib>
// #include <iostream>


// using namespace std;

class SonarFakePtCloud
{
public:
    SonarFakePtCloud(std::string _top) : topic(_top)
    {
        sonar_cloud_buffer;//.resize(1);
        initSonarPosition();
        initSonar();
    }

    void initSonar()
    {
        sonar_aux.header.frame_id = topic +"_range_frame";
        sonar_aux.points.resize(160);
        sonar_aux.channels.resize(1);
        sonar_aux.header.stamp = ros::Time::now();
        sensor_msgs::ChannelFloat32 ch;
        ch.name = std::string("distance");
        sonar_aux.channels[0] = ch;

        // sonar 0
        for(int i = 0; i < 20; i++)
        {
            sonar_aux.points.at(i) = positions[0];
        }
        // sonar 1
        for(int i = 20; i < 40; i++)
        {
            sonar_aux.points.at(i) = positions[1];
        }
        // sonar 2
        for(int i = 40; i < 60; i++)
        {
            sonar_aux.points.at(i) = positions[2];
        }
        // sonar 3
        for(int i = 60; i < 80; i++)
        {
            sonar_aux.points.at(i) = positions[3];
        }
        // sonar 4
        for(int i = 80; i < 100; i++)
        {
            sonar_aux.points.at(i) = positions[4];
        }
        // sonar 5
        for(int i = 100; i < 120; i++)
        {
            sonar_aux.points.at(i) = positions[5];
        }
        // sonar 6
        for(int i = 120; i < 140; i++)
        {
            sonar_aux.points.at(i) = positions[6];
        }
        // sonar 7
        for(int i = 140; i < 160; i++)
        {
            sonar_aux.points.at(i) = positions[7];
        }
        
        sonar_cloud_buffer = sonar_aux;
    }

    void initSonarPosition()
    {
        positions[0].x = 0.069;
        positions[0].y = 0.130;
        positions[0].z = 0.209;
        orientation_yaw[0] = 1.5708;

        positions[1].x = 0.110;
        positions[1].y = 0.110;
        positions[1].z = 0.209;
        orientation_yaw[1] = 0.8727;

        positions[2].x = 0.150;
        positions[2].y = 0.075;
        positions[2].z = 0.209;
        orientation_yaw[2] = 0.5235;

        positions[3].x = 0.160;
        positions[3].y = 0.034;
        positions[3].z = 0.209;
        orientation_yaw[3] = 0.1745;

        positions[4].x = 0.160;
        positions[4].y = -0.034;
        positions[4].z = 0.209;
        orientation_yaw[4] = -0.1745;

        positions[5].x = 0.150;
        positions[5].y = -0.075;
        positions[5].z = 0.209;
        orientation_yaw[5] = -0.5235;
    
        positions[6].x = 0.110;
        positions[6].y = -0.110;
        positions[6].z = 0.209;
        orientation_yaw[6] = -0.8727;

        positions[7].x = 0.069;
        positions[7].y = -0.130;
        positions[7].z = 0.209;
        orientation_yaw[7] = -1.5708;
    }

    geometry_msgs::Point32 getSonarPosition(int i)
    {
        return positions[i];
    }

    void updateBuffer(p2os_msgs::SonarArray sonarArray)
    {
        sonar_aux.header.frame_id = topic + "_range_frame";
        sensor_msgs::ChannelFloat32 ch;
        sonar_aux.header.stamp = ros::Time::now();
        // sonar 0
        for(int i = -10; i < 10; i++)
        {
            geometry_msgs::Point32 pos_aux;
            pos_aux = positions[0];
            pos_aux.x = sonarArray.ranges[0]*cos(((90.0+i)/180.0) * M_PI);
            pos_aux.y = sonarArray.ranges[0]*sin(((90.0+i)/180.0) * M_PI);

            sonar_aux.points.at(i+10) = pos_aux;
        }
        // sonar 1
        for(int i = -10; i < 10; i++)
        {
            geometry_msgs::Point32 pos_aux;
            pos_aux = positions[1];
            pos_aux.x = sonarArray.ranges[1]*cos(((50.0+i)/180.0) * M_PI);
            pos_aux.y = sonarArray.ranges[1]*sin(((50.0+i)/180.0) * M_PI);

            sonar_aux.points.at(i+30) = pos_aux;
        }
        // sonar 2
        for(int i = -10; i < 10; i++)
        {
            geometry_msgs::Point32 pos_aux;
            pos_aux = positions[2];
            pos_aux.x = sonarArray.ranges[2]*cos(((30.0+i)/180.0) * M_PI);
            pos_aux.y = sonarArray.ranges[2]*sin(((30.0+i)/180.0) * M_PI);

            sonar_aux.points.at(i+50) = pos_aux;
        }
        // sonar 3
        for(int i = -10; i < 10; i++)
        {
            geometry_msgs::Point32 pos_aux;
            pos_aux = positions[3];
            pos_aux.x = sonarArray.ranges[3]*cos(((10.0+i)/180.0) * M_PI);
            pos_aux.y = sonarArray.ranges[3]*sin(((10.0+i)/180.0) * M_PI);

            sonar_aux.points.at(i+70) = pos_aux;
        }
        // sonar 4
        for(int i = -10; i < 10; i++)
        {
            geometry_msgs::Point32 pos_aux;
            pos_aux = positions[4];
            pos_aux.x = sonarArray.ranges[4]*cos((-(10.0+i)/180.0) * M_PI);
            pos_aux.y = sonarArray.ranges[4]*sin((-(10.0+i)/180.0) * M_PI);

            sonar_aux.points.at(i+90) = pos_aux;
        }
        // sonar 5
        for(int i = -10; i < 10; i++)
        {
            geometry_msgs::Point32 pos_aux;
            pos_aux = positions[5];
            pos_aux.x = sonarArray.ranges[5]*cos((-(30.0+i)/180.0) * M_PI);
            pos_aux.y = sonarArray.ranges[5]*sin((-(30.0+i)/180.0) * M_PI);

            sonar_aux.points.at(i+110) = pos_aux;
        }
        // sonar 6
        for(int i = -10; i < 10; i++)
        {
            geometry_msgs::Point32 pos_aux;
            pos_aux = positions[6];
            pos_aux.x = sonarArray.ranges[6]*cos((-(50.0+i)/180.0) * M_PI);
            pos_aux.y = sonarArray.ranges[6]*sin((-(50.0+i)/180.0) * M_PI);

            sonar_aux.points.at(i+130) = pos_aux;
        }
        // sonar 7
        for(int i = -10; i < 10; i++)
        {
            geometry_msgs::Point32 pos_aux;
            pos_aux = positions[7];
            pos_aux.x = sonarArray.ranges[7]*cos((-(90.0+i)/180.0) * M_PI);
            pos_aux.y = sonarArray.ranges[7]*sin((-(90.0+i)/180.0) * M_PI);

            sonar_aux.points.at(i+150) = pos_aux;
        }

            sonar_cloud_buffer = sonar_aux;
    }

    sensor_msgs::PointCloud getCloud()
    {
        return sonar_cloud_buffer;
    }

    ~SonarFakePtCloud();

private:
    sensor_msgs::PointCloud sonar_aux;
    geometry_msgs::Point32 positions[8];
    double orientation_yaw[8];
    // std::vector<sensor_msgs::PointCloud> sonar_cloud_buffer;
    sensor_msgs::PointCloud sonar_cloud_buffer;

    std::string topic;
};
