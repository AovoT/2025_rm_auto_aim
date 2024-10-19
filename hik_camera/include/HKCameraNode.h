#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <mutex>
#include <memory>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <array>

#include "HK-Camera-Driver.h"
class HKCameraNode {
public:

    typedef std::shared_ptr<HKCameraNode> ptr;
    struct CameraInfo {
        std::array<double, 9> intrinsic_matrix;
        std::vector<double> distortion_vector;
    };
    static HKCameraNode& getInstance() {
        static HKCameraNode a = HKCameraNode();
        return a;
    }
    void init() {
        cam = new HK_Camera(&HKCameraNode::camCallBack, this);
        cam->cameraInit();
        cam->startCamera();
    }
    static void camCallBack(unsigned char *pData, MV_FRAME_OUT_INFO_EX *pFrameInfo, void *pUser) {
        std::lock_guard<std::mutex> lock(s_mutex);
        s_image = cv::Mat(pFrameInfo->nHeight, pFrameInfo->nWidth, CV_8UC3, pData);
    }
    CameraInfo getCameraInfo(const std::string& file_path) {
        YAML::Node node = YAML::LoadFile(file_path);
        CameraInfo camera_info;
        auto intrinsic_matrix = node["camera_matrix"]["data"];
        for (size_t i = 0; i < intrinsic_matrix.size(); i++) {
            camera_info.intrinsic_matrix[i] = intrinsic_matrix[i].as<double>();  
        }
        auto distortion_vector = node["distortion_coefficients"]["data"];
        for (size_t i = 0; i < distortion_vector.size(); i++) {
            camera_info.distortion_vector.push_back(distortion_vector[i].as<double>());
        }
        return camera_info;
    }
    cv::Mat getImage() {
        std::lock_guard<std::mutex> lock(s_mutex);
        return s_image.clone();
    }
private:
    HKCameraNode() {init();}
    static cv::Mat s_image;
    static std::mutex s_mutex;
    HK_Camera* cam;
};


