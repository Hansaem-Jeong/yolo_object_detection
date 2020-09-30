#include <yolo_object_detection/yolo_object_detection_node.hpp>
#include <iostream>
#include <opencv2/imgproc.hpp>



namespace yolo_object_detection
{

static std::vector<std::string> objects_names_from_file(std::string const filename)
{
    std::ifstream file(filename);
    std::vector<std::string> file_lines;
    if (!file.is_open()) return file_lines;
    for(std::string line; getline(file, line);) file_lines.push_back(line);
    std::cout << "object names loaded \n";

    return file_lines;
}

YoloObjectDetectionNode::YoloObjectDetectionNode()
    : privateNh("~"),
      it(nh)
{
    std::string yolo_config_file, yolo_names_file, yolo_weights_file;

    privateNh.getParam("yolo_names_file", yolo_names_file);
    privateNh.getParam("yolo_config_file", yolo_config_file);
    privateNh.getParam("yolo_weights_file", yolo_weights_file);

    yoloDetector = new Detector(yolo_config_file, yolo_weights_file, 0.2f/* thresh*/);
    objectsNames = objects_names_from_file(yolo_names_file);

    rgbSub = it.subscribe("/image_color", 1, &YoloObjectDetectionNode::imageCallback, this);
}

YoloObjectDetectionNode::~YoloObjectDetectionNode()
{
    delete yoloDetector;
}

void YoloObjectDetectionNode::imageCallback(const sensor_msgs::ImageConstPtr& im_msg)
{
    cv_bridge::CvImageConstPtr cv_ptr_im = cv_bridge::toCvShare(im_msg);

    cv::Mat color = cv_ptr_im->image.clone();
    cv::cvtColor(color, color, cv::COLOR_RGB2BGR);

    objects.clear();
    objects = yoloDetector->detect(color);

    cv::Mat detection = color.clone();
    drawBoxes(detection);

    //cv::imshow("color", color);
    cv::imshow("detection", detection);

    cv::waitKey(1);
}

void YoloObjectDetectionNode::run()
{
    ros::spin();
}

void YoloObjectDetectionNode::drawBoxes(cv::Mat mat_img)
{
    int const colors[6][3] = { { 1,0,1 },{ 0,0,1 },{ 0,1,1 },{ 0,1,0 },{ 1,1,0 },{ 1,0,0 } };

    for(auto &i : objects)
    {
        cv::Scalar color = obj_id_to_color(i.obj_id);
        cv::rectangle(mat_img, cv::Rect(i.x, i.y, i.w, i.h), color, 2);

        if(objectsNames.size() > i.obj_id)
        {
            std::string obj_name = objectsNames[i.obj_id];
            if (i.track_id > 0) obj_name += " - " + std::to_string(i.track_id);
            cv::Size const text_size = getTextSize(obj_name, cv::FONT_HERSHEY_COMPLEX_SMALL, 1.2, 2, 0);
            int max_width = (text_size.width > i.w + 2) ? text_size.width : (i.w + 2);
            max_width = std::max(max_width, (int)i.w + 2);

            cv::rectangle(mat_img, cv::Point2f(std::max((int)i.x - 1, 0), std::max((int)i.y - 35, 0)),
                          cv::Point2f(std::min((int)i.x + max_width, mat_img.cols - 1), std::min((int)i.y, mat_img.rows - 1)), color, CV_FILLED, 8, 0);
            putText(mat_img, obj_name, cv::Point2f(i.x, i.y - 16), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.2, cv::Scalar(0, 0, 0), 2);
        }
    }
}

} // namespace yolo_object_detection
