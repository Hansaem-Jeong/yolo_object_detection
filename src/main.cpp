#include <ros/ros.h>
#include <yolo_object_detection/yolo_object_detection_node.hpp>

using namespace yolo_object_detection;



int main(int argc, char** argv)
{
  ros::init(argc, argv, "yolo_object_detection_node");

  YoloObjectDetectionNode node;

  node.run();
  
  return 0;
}
