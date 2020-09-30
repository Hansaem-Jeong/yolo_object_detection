#ifndef YOLO_OBJECT_DETECTION_NODE_HPP
#define YOLO_OBJECT_DETECTION_NODE_HPP

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "yolo_v2_class.hpp"



namespace yolo_object_detection
{

class YoloObjectDetectionNode
{

public:
    YoloObjectDetectionNode();
    ~YoloObjectDetectionNode();
    void imageCallback(const sensor_msgs::ImageConstPtr& msg_rgb);
    void run();
    void drawBoxes(cv::Mat mat_img);

private:
    ros::NodeHandle nh, privateNh;
    image_transport::ImageTransport it;
    image_transport::Subscriber rgbSub;
    Detector *yoloDetector; // use smart ptr instead
    std::vector<std::string> objectsNames;
    std::vector<bbox_t> objects;
};

} // namespace yolo_object_detection

#endif // YOLO_OBJECT_DETECTION_NODE_HPP
