#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float64MultiArray.h"

ros::Publisher dist_angle_pub;


void semaphoreCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

void velocityCallback(const std_msgs::Float64MultiArray::ConstPtr& array)
{

    double distance = array->data[0];
    double angle = array->data[1];

    /// Publish Distance and Angle Info
    std_msgs::Float64MultiArray array_out;
    array_out.data.clear();
    array_out.data.push_back(distance);         // actual distance
    array_out.data.push_back(angle);            // actual angle
    array_out.data.push_back(0.5);              // reference velocity
    array_out.data.push_back(0.4);              // reference distance
    array_out.data.push_back(0);                // reference angle
    ros::Rate loop_rate(100);
    dist_angle_pub.publish(array_out);
    ros::spinOnce();
    loop_rate.sleep();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "conde_decision");

  ros::NodeHandle n("~");

  ros::Subscriber sub2 = n.subscribe("/conde_dist_angle", 100, velocityCallback);
  ros::Subscriber sub3 = n.subscribe("/conde_signalling_panel_info", 100, semaphoreCallback);

  dist_angle_pub = n.advertise<std_msgs::Float64MultiArray>("/conde_msg", 1);

  ros::spin();

  return 0;
}
