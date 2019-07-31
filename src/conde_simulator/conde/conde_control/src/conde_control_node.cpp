#include "ros/ros.h"

#include "std_msgs/String.h"
#include "std_msgs/Float64.h"

#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float64MultiArray.h"

#include <string>

ros::Publisher vel_pub;
//ros::NodeHandle *vel_node;
geometry_msgs::Twist velocityToSend;

/// Callback References
void sensorCallback(const std_msgs::Float64MultiArray::ConstPtr& array)
{
    std::vector<double> data(5);

    data[0] = array->data[0];       // Actual Distance (m)
    data[1] = array->data[1];       // Actual Angle (rad)
    data[2] = array->data[2];       // Vlinear (m/s)
    data[3] = array->data[3];       // Reference Distance (m)
    data[4] = array->data[4];       // Reference Angle (rad)
	
    double vlinear = 0;
    double w = 0;

    velocityToSend.angular.x = 0;
    velocityToSend.angular.y = 0;
    velocityToSend.angular.z = w;

    velocityToSend.linear.x = vlinear;
    velocityToSend.linear.y = 0;
    velocityToSend.linear.z = 0;

    ros::Rate loop_rate(50);
    vel_pub.publish(velocityToSend);
    ros::spinOnce();
    loop_rate.sleep();
}



int main(int argc, char **argv) {

    ros::init(argc,argv,"conde_control");
    
    ros::NodeHandle vel_node;
    
    std::string ref_topic = std::string("/conde_msg");
    ros::Subscriber sub1 = vel_node.subscribe(ref_topic, 50, sensorCallback);

    //vel_node = new ros::NodeHandle("~");
    vel_pub = vel_node.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    ros::spin();

    return 0;

}
