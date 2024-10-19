#include "../include/HKCameraNode.h"
#include <mutex>
#include <opencv4/opencv2/core/utility.hpp>

cv::Mat HKCameraNode::s_image = cv::Mat();
std::mutex HKCameraNode::s_mutex;