#ifndef HK_CAMERA_DRIVER_HK_CAMERA_DRIVER_H
#define HK_CAMERA_DRIVER_HK_CAMERA_DRIVER_H

#include <chrono>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <iostream>
#include "MvCameraControl.h"
#include <opencv2/opencv.hpp>

using CallbackType = void (*)(unsigned char*, MV_FRAME_OUT_INFO_EX*, void*);

// void camCallBack(unsigned char *pData, MV_FRAME_OUT_INFO_EX *pFrameInfo, void *pUser);

class HK_Camera {
public:
    HK_Camera();
    HK_Camera(CallbackType callback, void* HKCameraNode);
    ~HK_Camera();
    unsigned int findConnectableUSBDevice();
    void cameraInit();
    void startCamera();
    void stopCamera();
    bool _is_open;
private:
    MV_CC_DEVICE_INFO_LIST _device_list;
    MV_CC_DEVICE_INFO* _device_info;
    CallbackType _call_back_ptr;
    void* handle; // 相机句柄
    void* _hik_camera_node;
    int nRet; // 状态码（通用）
};



#endif //HK_CAMERA_DRIVER_HK_CAMERA_DRIVER_H
