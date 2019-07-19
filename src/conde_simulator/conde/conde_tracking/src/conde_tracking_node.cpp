#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/opencv_modules.hpp>
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/String.h"
#include <algorithm>
#include "geometry_msgs/Twist.h"


#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Bool.h"

bool ipmLeftDone;
bool ipmRightDone;

/// Intrisic Camera parameters
/*
    [   alpha_x ,   beta    ,   x0
        0       ,   alpha_Y ,   y0
        0       ,   0       ,   1   ]
    alpha_x, alpha_y -> focal length
    x0, y0           -> principal point
*/
//------------------------Camera right parameters----------------------------------------------
cv::Mat cameraRight_intrinsic = (cv::Mat_<double>(3,3) << \
                                 130.81305          , 0                     , 79.81980 ,\
                                 0                  , 131.22421             , 58.91209,\
                                 0                  , 0                     , 1);

cv::Mat cameraRight_T_chess_robot = (cv::Mat_<double>(4,4) <<  \
                                     -1,    0,  0,  0.612,\
                                     0,   -1,  0,  0.0,\
                                     0,    0,  1,  -0.004,\
                                     0,    0,  0,  1);

cv::Mat cameraRight_T_cam_chess = (cv::Mat_<double>(4,4) << \
                                   0.330007,      0.918154,       0.219294,     -0.540327012,\
                                   0.562501,     -0.004706,      -0.826783,     -0.018409465,\
                                   -0.758082,      0.396197,      -0.518016,      1.038223574,\
                                   0       ,      0       ,       0       ,      1);
// Distortion coeficients
cv::Mat cameraRight_dist_coef = (cv::Mat_<double>(1,4) << -0.275678598507515 , 0.045106260288961 ,
                                 0.004883645512607 , 0.001092737340199);

//------------------------Camera left parameters----------------------------------------------

cv::Mat cameraLeft_intrinsic = (cv::Mat_<double>(3,3) << \
                                132.31872          , 0                     , 74.70743 ,\
                                0                  , 132.17822             , 52.77469,\
                                0                  , 0                     , 1);

cv::Mat cameraLeft_T_chess_robot = (cv::Mat_<double>(4,4) <<  \
                                    -1,    0,  0,  0.633,\
                                    0,   -1,  0,  0.72,\
                                    0,    0,  1,  -0.004,\
                                    0,    0,  0,  1);

cv::Mat cameraLeft_T_cam_chess = (cv::Mat_<double>(4,4) << \
                                  -0.436125,      0.850786,      -0.293187,     0.081835848,\
                                  0.553047,     -0.003607,      -0.833142,     0.014687224,\
                                  -0.709883,     -0.525501,      -0.468951,     1.363192844,\
                                  0       ,      0       ,       0       ,     1);
// Distortion coeficients
cv::Mat cameraLeftt_dist_coef = (cv::Mat_<double>(1,4) << -0.275678598507515 , 0.045106260288961 ,
                                 0.004883645512607 , 0.001092737340199);
//--------------------------------------------------------------------------------------------

ros::Publisher dist_angle_pub;
ros::Publisher crossWalk_pub;

///--------------------------------------------------------------------------------------------------
void imageMergeAndTrack()
{
	double distIPM = 0.375;// meters
	double angleIPM = 0; // rad
	
	/// Publish Distance and Angle Info
	std_msgs::Float64MultiArray array;
	array.data.clear();
	array.data.push_back(distIPM);
	array.data.push_back(angleIPM);//angleIPM
	ros::Rate loop_rate(200);
	dist_angle_pub.publish(array);
	ros::spinOnce();
	loop_rate.sleep();

}

///------------------------------------------------LEFT--------------------------------------------------
void imageLeftCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv::Mat img_rgb = cv_bridge::toCvShare(msg, "bgr8")->image;
        
        cv::imshow("left rgb", img_rgb);
        
        ipmLeftDone = true;
        
        uint8_t k = cv::waitKey(1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }

    if(ipmRightDone && ipmLeftDone)
    {
        ipmLeftDone = false;
        ipmRightDone = false;
        imageMergeAndTrack();
    }

}
///----------------------------------------------RIGHT-----------------------------------------------
void imageRightCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv::Mat img_rgb = cv_bridge::toCvShare(msg, "bgr8")->image;
        
        cv::imshow("right rgb", img_rgb);
        
        ipmRightDone = true;
        
        uint8_t k = cv::waitKey(1);

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }

    if(ipmRightDone && ipmLeftDone)
    {
        ipmLeftDone = false;
        ipmRightDone = false;
        imageMergeAndTrack();
    }
    return;
}

///--------------------------------------------------------------------------------------------------
///--------------------------------------------------------------------------------------------------
int main(int argc, char** argv)
{

    /// init variables
    ros::init(argc, argv, "conde_tracking_node");
    ros::NodeHandle nh("~");
    cv::namedWindow("right rgb");
    cv::namedWindow("left rgb");

    std::string cameraRightTopic;
    std::string cameraLeftTopic;

    if(!(nh.getParam("camera_right", cameraRightTopic))){
        std::cerr << "Parameter (camera_right) not found" << std::endl;
        return 0;
    }

    std::cout << "Parameter camera_right: " << cameraRightTopic <<  std::endl;

    if(!(nh.getParam("camera_left", cameraLeftTopic))){
        std::cout << "Parameter (camera_left) not found" << std::endl;
        return 0;
    }
    std::cout << "Parameter camera_left: " << cameraLeftTopic <<  std::endl;

    cv::startWindowThread();
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber subRight = it.subscribe(cameraRightTopic, 1, imageRightCallback);
    image_transport::Subscriber subLeft = it.subscribe(cameraLeftTopic, 1, imageLeftCallback);

    dist_angle_pub = nh.advertise<std_msgs::Float64MultiArray>("/conde_dist_angle", 1);

    ros::spin();
    cv::destroyWindow("view");
}

