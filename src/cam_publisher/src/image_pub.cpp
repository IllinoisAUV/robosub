#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream> // for converting the command line parameter to integer

#define IMAGE_TOPIC "/camera/image_raw"
// #define IMAGE_TOPIC_LEFT "/camera/left/image_raw"
// #define IMAGE_TOPIC_RIGHT "/camera/right/image_raw"
// #define IMAGE_TOPIC_DOWN "/camera/down/image_raw"

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
    // image_transport::Publisher leftPub = it.advertise(IMAGE_TOPIC_LEFT, 10);
    // image_transport::Publisher rightPub = it.advertise(IMAGE_TOPIC_RIGHT, 10);
    // image_transport::Publisher downPub = it.advertise(IMAGE_TOPIC_DOWN, 10);

    // Convert the passed as command line parameter index for the video device to an integer
    // std::istringstream video_sourceCmd1(argv[1]);
    // std::istringstream video_sourceCmd2(argv[2]);
    // std::istringstream video_sourceCmd3(argv[3]);
    // int video_source_1;
    // int video_source_2;
    // int video_source_3;
    // // Check if it is indeed a number
    // if(!(video_sourceCmd1 >> video_source_1) || !(video_sourceCmd2 >> video_source_2) || !(video_sourceCmd3 >> video_source_3))
    // {
    //     ROS_ERROR("No video source");
    //     return 1;
    // }

    cv::VideoCapture cap(argv[1]);
    // cv::VideoCapture capLeft(video_source_1);
    // cv::VideoCapture capRight(video_source_2);
    // cv::VideoCapture capDown(video_source_3);
    // Check if video device can be opened with the given index
    // if(!capLeft.isOpened() || !capRight.isOpened() || !capDown.isOpened()) {
    //     ROS_ERROR("No opened capture");
    //     return 1;
    // }
    if(!cap.isOpened()) {
        ROS_ERROR("No video source");
        return 1;
    }


    cv::Mat Frame;
    // cv::Mat leftFrame;
    // cv::Mat rightFrame;
    // cv::Mat downFrame;

    sensor_msgs::ImagePtr msg;

    ros::Rate loop_rate(5);
    while (ros::ok()) {
        cap >> Frame;
        // capLeft >> leftFrame;
        // capDown >> downFrame;
	    // capRight >> rightFrame;
        // Check if grabbed frame is actually full with some content
        if(!Frame.empty()) {
            msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", Frame).toImageMsg();
            camPub.publish(msg);
	        Frame.release();
        }
        // if(!leftFrame.empty()) {
        //     msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", leftFrame).toImageMsg();
        //     leftPub.publish(msg);
	    // leftFrame.release();
        // }
	    // if(!rightFrame.empty()){
        //     msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rightFrame).toImageMsg();
        //     rightPub.publish(msg);
	    // rightFrame.release();
	    // }
        // if( !downFrame.empty()){
        //     msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", downFrame).toImageMsg();
        //     downPub.publish(msg);
        //     downFrame.release();
        // }
        ros::spinOnce();
        loop_rate.sleep();
    }
}
