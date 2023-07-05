#include <fstream>
#include <iomanip>
#include <thread>
#include <sstream>
#include <cmath>
#include <prism/s6/ls2d_device.h>
#include <prism/s6/sensor_search.h>
#include <prism/utils/timer.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/PointCloud2.h"
#include "ls2d/Sensor_info.h"

const int POINTS_IN_ARRAY = 1024;
const int SIZEOF_FLOAT    = 4;
const std::string sensor_search_addr = "192.168.1.255";
const std::string sensor_ip_addr     = "192.168.1.146";

using namespace prism::s6;
using namespace std;

sensor_search sens_search;
ls2d_device sens;


bool connect(ros::NodeHandle& n){
    ROS_INFO("Connecting to IP [%s] ...", sensor_ip_addr.c_str() );
    if (!sens.connect(sensor_ip_addr)) {
        ROS_INFO("FAILED");
        return false;
    }
    else {
        ROS_INFO("CONNECTED");
    }  
    ros::Publisher sensor_info_pub = n.advertise<ls2d::Sensor_info>("sensor_info", 10);
    
    ls2d::Sensor_info msg;
    auto info = sens.info();
    msg.header.stamp    = ros::Time::now();
    msg.header.frame_id = "/sensor_base_link";
    msg.ip              = sens.info().ip_addr;
    msg.descrioption    = sens.info().description;
    msg.serial          = sens.info().serial;
    msg.firmware_ver    = sens.info().firmware_ver;
    msg.fractional_bits = sens.info().fractional_bits;

    sensor_info_pub.publish(msg);
    return true;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "ls2d_sensor");
    ros::NodeHandle n;

    if (connect(n)){
        ros::Rate loop_rate(30);  // SET FREC HERE

        ros::Publisher stream_pub = n.advertise<sensor_msgs::PointCloud2>("point_cloud", 100000);

        result_info info;
        std::array<data_point, POINTS_IN_ARRAY> points; 
        sensor_msgs::PointCloud2 cloud_msg;

        std::vector<unsigned char> vector_char;
        vector_char.resize(SIZEOF_FLOAT * POINTS_IN_ARRAY * 3);

        std::vector<sensor_msgs::PointField> field_vector_(3);

        field_vector_[0].name     = "x";
        field_vector_[0].offset   = 0;
        field_vector_[0].datatype = sensor_msgs::PointField::FLOAT32; 
        field_vector_[0].count    = 1;

        field_vector_[1].name     = "y";
        field_vector_[1].offset   = SIZEOF_FLOAT;
        field_vector_[1].datatype = sensor_msgs::PointField::FLOAT32; 
        field_vector_[1].count    = 1;

        field_vector_[2].name     = "z";
        field_vector_[2].offset   = SIZEOF_FLOAT * 2;
        field_vector_[2].datatype = sensor_msgs::PointField::FLOAT32; 
        field_vector_[2].count    = 1;

        while (ros::ok()){
            sens.stream_enable(true);

            this_thread::sleep_for(10ms);
            sens.stream_read(&info, points.data());
            int offset = 0;
            for (unsigned point_idx = 0; point_idx < POINTS_IN_ARRAY; ++point_idx) {
                data_point point = points[point_idx];

                memcpy(&vector_char[offset + SIZEOF_FLOAT * 0], &point.x, SIZEOF_FLOAT);
                memcpy(&vector_char[offset + SIZEOF_FLOAT * 1], &point.y, SIZEOF_FLOAT);
                memcpy(&vector_char[offset + SIZEOF_FLOAT * 2], &point.z, SIZEOF_FLOAT);
        
                offset += 3 * SIZEOF_FLOAT;
            }

            cloud_msg.data  = vector_char;
            cloud_msg.header.stamp    = ros::Time::now();
            cloud_msg.header.frame_id = "/sensor_base_link";  
            cloud_msg.height          = 1;
            cloud_msg.width           = POINTS_IN_ARRAY;   
            cloud_msg.fields          = field_vector_;
            cloud_msg.is_bigendian    = false;  
            cloud_msg.point_step      = 3 * SIZEOF_FLOAT;
            cloud_msg.row_step        = 3 * SIZEOF_FLOAT  * POINTS_IN_ARRAY;
            cloud_msg.is_dense        = true; 

            stream_pub.publish(cloud_msg);
            ROS_INFO("Cloud published!");  // debug

            sens.stream_enable(false);
  
            ros::spinOnce();
            loop_rate.sleep();
        }   

    }
    else  {ROS_INFO("Connection false, please, retry. Exit.");}
    return 0;
 };