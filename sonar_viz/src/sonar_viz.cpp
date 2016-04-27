#include "ros/ros.h"
#include "p2os_msgs/SonarArray.h"
#include "sensor_msgs/PointCloud.h"
#include "sonar_viz/SonarRange.h"
#include "sonar_viz/SonarFakePtCloud.h"


using namespace std;

SonarRange *sonar;
SonarFakePtCloud *sonar_cloud;

void sonarUpdate(const p2os_msgs::SonarArray::ConstPtr& msg)
{
    sonar->updateBuffer(*msg);
    sonar_cloud->updateBuffer(*msg);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "sonar_sensor_viz");
    ros::NodeHandle n;
    string topic_range = "/sonar";
    string topic_point_cloud = "/back_sonar";
    ros::Publisher sonar_pub = n.advertise<sensor_msgs::Range>("sonar_range", 1000);
    ros::Publisher cloud_pub = n.advertise<sensor_msgs::PointCloud>("sonar_cloud", 1000);
    ros::Subscriber sonar_sub = n.subscribe(topic_range, 1000, sonarUpdate);

    sonar = new SonarRange(topic_range);
    sonar_cloud = new SonarFakePtCloud(topic_point_cloud);

    ros::Rate loop_rate(5);

    loop_rate.sleep();

    while(ros::ok())
    {
        std::vector<sensor_msgs::Range> sonar_values = sonar->getSonar();
        sensor_msgs::PointCloud sonar_cloud_values = sonar_cloud->getCloud();
        // std::vector<sensor_msgs::PointCloud> sonar_cloud_values = sonar_cloud->getCloud();


        for (unsigned int i = 0; i < sonar_values.size(); ++i)
        {
            sonar_pub.publish(sonar_values.at(i));
        }

        cloud_pub.publish(sonar_cloud_values);

        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
