#include "sensor_msgs/Range.h"
#include "p2os_msgs/SonarArray.h"
#include <boost/lexical_cast.hpp>
#include <cmath>
#include <string>
#include <vector>
#include <cstdlib>

// using namespace std;

class SonarRange
{
public:
    SonarRange(std::string _topic) : topic(_topic)
    {
        sonar_range_buffer.resize(8);
        initSonar();
    }

    void initSonar()
    {
        for(unsigned int i = 0; i < sonar_range_buffer.size(); i++)
        {
            sonar_aux.radiation_type = sensor_msgs::Range::ULTRASOUND;
            sonar_aux.field_of_view = (20.0/180.0) * M_PI;
            sonar_aux.min_range = 0.1;
            sonar_aux.max_range = 5.0;
            sonar_aux.range = 5.0;
            sonar_aux.header.stamp = ros::Time::now();
            sonar_aux.header.frame_id = topic + '_' + boost::lexical_cast<std::string>(i) + "_frame";

            sonar_range_buffer.at(i) = sonar_aux;
        }
    }

    void updateBuffer(p2os_msgs::SonarArray sonarArray)
    {
        for(unsigned int i = 0; i < sonar_range_buffer.size(); i++)
        {
            sonar_aux.radiation_type = sensor_msgs::Range::ULTRASOUND;
            sonar_aux.field_of_view = (20.0/180.0) * M_PI;
            sonar_aux.min_range = 0.1;
            sonar_aux.max_range = 5.0;
            sonar_aux.range = sonarArray.ranges[i];
            sonar_aux.header.stamp = ros::Time::now();
            sonar_aux.header.frame_id = topic + '_' + boost::lexical_cast<std::string>(i) + "_frame";

            sonar_range_buffer.at(i) = sonar_aux;
        }
    }

    std::vector<sensor_msgs::Range> getSonar()
    {
        return sonar_range_buffer;
    }

    ~SonarRange() {}

private:

    std::vector<sensor_msgs::Range> sonar_range_buffer;
    sensor_msgs::Range sonar_aux;

    std::string topic;
};
