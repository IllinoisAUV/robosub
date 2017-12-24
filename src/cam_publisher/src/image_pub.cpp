#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream> // for converting the command line parameter to integer

#define IMAGE_TOPIC "/camera/image_raw"

int main(int argc, char** argv)
{
    // Check if video source has been passed as a parameter
    if(argc != 2) {
        ROS_ERROR("No video source specified");
        return 1;
    }

    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher camPub = it.advertise(IMAGE_TOPIC, 10);

    cv::VideoCapture cap(argv[1]);
    if(!cap.isOpened()) {
        ROS_ERROR("No video source");
        return 1;
    }
    
    cv::Mat Frame;
    sensor_msgs::ImagePtr msg;

    ros::Rate loop_rate(5);
    while (ros::ok()) {
        cap >> Frame;
        if(!Frame.empty()) {
            msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", Frame).toImageMsg();
            camPub.publish(msg);
	        Frame.release();
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}
