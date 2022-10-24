#include <iostream>
#include <yaml-cpp/yaml.h>
#include "opencv2/opencv.hpp"
#include <vector>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include "hikrobot_camera.hpp"

// 剪裁掉照片和雷达没有重合的视角，去除多余像素可以使rosbag包变小
#define FIT_LIDAR_CUT_IMAGE false
#if FIT_LIDAR_CUT_IMAGE
    #define FIT_min_x 420
    #define FIT_min_y 70
    #define FIT_max_x 2450
    #define FIT_max_y 2000
#endif 

using namespace std;
using namespace cv;


int main(int argc, char **argv)
{
    //********** variables    **********/
    cv::Mat src;
    //string src = "",image_pub = "";
    //********** rosnode init **********/
    ros::init(argc, argv, "hikrobot_camera");
    ros::NodeHandle hikrobot_camera;
    camera::Camera MVS_cap(hikrobot_camera);
    //********** rosnode init **********/
    image_transport::ImageTransport main_cam_image(hikrobot_camera);
    image_transport::CameraPublisher image_pub = main_cam_image.advertiseCamera("/hikrobot_camera/image_raw", 1000);

    sensor_msgs::Image image_msg;
    sensor_msgs::CameraInfo camera_info_msg;
    cv_bridge::CvImagePtr cv_ptr = boost::make_shared<cv_bridge::CvImage>();
    cv_ptr->encoding = sensor_msgs::image_encodings::BGR8;  // 就是rgb格式 
    MVS_cap.set(camera::CAP_PROP_TRIGGER_MODE, 1);
    MVS_cap.set(camera::CAP_PROP_TRIGGER_SOURCE, 0);
    MVS_cap.set(camera::CAP_PROP_FRAMERATE_ENABLE, false);


    camera_info_msg.width = 2048;
    camera_info_msg.height = 1536;
    camera_info_msg.distortion_model = "plumb_bob";
    std::vector<double> d{-0.217529, 0.169305, 0.005223999999999999, -0.001093, 0};
    camera_info_msg.D = d;
    boost::array<double, 9> k = {2299.303861, 0, 997.019631, 0, 2301.232046, 815.269991, 0, 0, 1};
    camera_info_msg.K = k;
    boost::array<double, 12> p = {2208.608154, 0, 992.949178, 0, 0, 2244.101074, 821.213246, 0, 0, 0, 1, 0};
    camera_info_msg.P = p;
    boost::array<double, 9> r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    camera_info_msg.R = r;

 

    //********** 40 Hz        **********/
    ros::Rate loop_rate(40);

    int index = 0;
    while (ros::ok())
    {
        loop_rate.sleep();
        ros::spinOnce();

        MVS_cap.ReadImg(src);
        if (src.empty())
        {
            continue;
        }
        cv_ptr->image = src;
        MV_FRAME_OUT_INFO_EX frameInfo = MVS_cap.getFrameInfo();

        image_msg = *(cv_ptr->toImageMsg());
        uint64_t time = (uint64_t) frameInfo.nDevTimeStampHigh << 32 | frameInfo.nDevTimeStampLow;
        uint64_t sec = time/1000000000;
        uint64_t nsec = time%1000000000;
        image_msg.header.stamp = ros::Time(sec , nsec);  // ros发出的时间不是快门时间
        image_msg.header.frame_id = "hikrobot_camera";

        camera_info_msg.header.frame_id = image_msg.header.frame_id;
	    camera_info_msg.header.stamp = image_msg.header.stamp;

        image_pub.publish(image_msg, camera_info_msg);

        //*******************************************************************************************************************/
    }
    return 0;
}
