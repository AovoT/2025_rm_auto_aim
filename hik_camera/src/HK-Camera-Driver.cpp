#include "../include/HK-Camera-Driver.h"

void camCallBack(unsigned char *pData, MV_FRAME_OUT_INFO_EX *pFrameInfo, void *pUser) {

    static int count = 0;
    static auto start = std::chrono::high_resolution_clock::now();

    count++;
    auto now = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = now - start;

    if (elapsed.count() >= 1.0) { // 每秒输出一次
        std::cout << "fps : " << count << std::endl;
        count = 0;
        start = now;
    }

    cv::Mat img = cv::Mat(pFrameInfo->nHeight, pFrameInfo->nWidth, CV_8UC3, pData);
    cv::cvtColor(img, img, cv::COLOR_RGB2BGR);
    cv::imshow("cap_img", img);
    cv::waitKey(1);

}

HK_Camera::HK_Camera() {
    _call_back_ptr = camCallBack;
    _hik_camera_node = nullptr;
    handle = nullptr;
    _device_info = nullptr;
}

HK_Camera::HK_Camera(CallbackType callback, void* HKCameraNode = nullptr) {
    _call_back_ptr = callback;
    _hik_camera_node = HKCameraNode;
    handle = nullptr;
    _device_info = nullptr;
}

HK_Camera::~HK_Camera() {
    MV_CC_StopGrabbing(handle);
    MV_CC_CloseDevice(handle);
}

void HK_Camera::stopCamera() {
    MV_CC_StopGrabbing(handle);
    MV_CC_CloseDevice(handle);
}

unsigned int HK_Camera::findConnectableUSBDevice() {
    int nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &_device_list);
    if (_device_list.nDeviceNum > 0) {
        std::cout << "发现 " << _device_list.nDeviceNum << " 个设备 : " <<std::endl;
        for (unsigned int i = 0; i < _device_list.nDeviceNum; i++) {
            _device_info = _device_list.pDeviceInfo[i];
            if (_device_info == nullptr) break;
            std::cout << "--------------------------------------------" << "\n第 " << i + 1 << " 个 : " << std::endl;
            if (_device_info->nTLayerType == MV_USB_DEVICE && MV_CC_IsDeviceAccessible(_device_info, MV_ACCESS_Monitor)) {
                std::cout << "idProduct : " << _device_info->SpecialInfo.stUsb3VInfo.idProduct << std::endl;
                std::cout << "idVendor : " << _device_info->SpecialInfo.stUsb3VInfo.idVendor << std::endl;
                std::cout << "chDeviceGUID : " << _device_info->SpecialInfo.stUsb3VInfo.chDeviceGUID << std::endl;
                std::cout << "chVendorName : " << _device_info->SpecialInfo.stUsb3VInfo.chVendorName << std::endl;
                std::cout << "chModelName : " << _device_info->SpecialInfo.stUsb3VInfo.chModelName << std::endl;
                std::cout << "chDeviceVersion : " << _device_info->SpecialInfo.stUsb3VInfo.chDeviceVersion << std::endl;
                std::cout << "chSerialNumber : " << _device_info->SpecialInfo.stUsb3VInfo.chSerialNumber << std::endl;
                std::cout << "nDeviceAddress : " << _device_info->SpecialInfo.stUsb3VInfo.nDeviceAddress << std::endl;
            }
            std::cout << "--------------------------------------------" << std::endl;
        }
        if (_device_list.nDeviceNum > 1) {
            int index = -1;
            while (index < 1 && index > _device_list.nDeviceNum) {
                std::cout << "请输入您要连接的设备索引号 : ( 1 到 " << _device_list.nDeviceNum << " )" << std::endl;
                std::cin >> index;
            }
            _device_info = _device_list.pDeviceInfo[index - 1];
        } else {
            _device_info = _device_list.pDeviceInfo[0];
        }
    } else {
        std::cout << "未找到可用设备" << std::endl;
    }
    return _device_list.nDeviceNum;
}
void HK_Camera::cameraInit() {

    int test_num = 3;
    while (_device_info == nullptr && test_num--) {
        findConnectableUSBDevice();
        sleep(1);
    }
    if (_device_info == nullptr) return;

    MV_CC_CreateHandle(&handle, _device_info);
    MV_CC_OpenDevice(handle);


    MVCC_INTVALUE_EX nWidthMaxValue = {0}, nHeightMaxValue = {0};
    //如需获取当前相机图像宽高，需要将WidthMax替换成Width
    nRet = MV_CC_GetIntValueEx(handle, "WidthMax", &nWidthMaxValue);
    if (MV_OK != nRet) {
        printf("Get WidthMax fail! nRet [0x%x]\n", nRet);
    }
    //如需获取当前相机图像宽高，需要将HeightMax替换成Height
    nRet = MV_CC_GetIntValueEx(handle, "HeightMax", &nHeightMaxValue);
    if (MV_OK != nRet) {
        printf("Get HeightMax fail! nRet [0x%x]\n", nRet);
    }

    // 设置图像像素
    nRet = MV_CC_SetIntValue(handle, "Width", nWidthMaxValue.nCurValue);
    if (MV_OK != nRet) {
        printf("error: Set Img Width fail [%x]\n", nRet);
    }
    nRet = MV_CC_SetIntValue(handle, "Height", nHeightMaxValue.nCurValue);
    if (MV_OK != nRet) {
        printf("error: Set Img Height fail [%x]\n", nRet);
    }

    //帧率控制使能，true表示打开，false标识关闭
    nRet = MV_CC_SetBoolValue(handle, "AcquisitionFrameRateEnable", true);
    if (nRet != MV_OK) {
        printf("Warning: Set AcquisitionBurstFrameCountfail nRet [0x%x]!\n", nRet);
    }
    //设置相机帧率，需注意不要超过相机支持的最大的帧率（相机规格书），超过了也没有意义（需要注意的是不同的像素类型支持的帧率也不同）
    nRet = MV_CC_SetFloatValue(handle, "AcquisitionFrameRate", 25);
    if (nRet != MV_OK) {
        printf("Warning: Set AcquisitionBurstFrameCountfail nRet [0x%x]!\n", nRet);
    }


    //设置手动曝光，设置曝光时间
    nRet = MV_CC_SetEnumValue(handle, "ExposureMode",0);
    nRet = MV_CC_SetFloatValue(handle, "ExposureTime", 10000);
    if (MV_OK != nRet) {
        printf("Set ExposureTime fail nRet [0x%x]!\n", nRet);
    }
    //设置自动曝光
    nRet = MV_CC_SetEnumValue(handle, "ExposureAuto", 0); // 0：off 1：once 2：Continuous
    if (MV_OK != nRet) {
        printf("Set ExposureAuto fail nRet [0x%x]!\n", nRet);
    }


    //模拟增益设置
    nRet = MV_CC_SetFloatValue(handle, "Gain", 1);
    if (MV_OK != nRet) {
        printf("Set Gain fail nRet [0x%x]\n", nRet);
    }
    //设置自动增益
    nRet = MV_CC_SetEnumValue(handle, "GainAuto", 1);
    if (MV_OK != nRet) {
        printf("Set GainAuto fail nRet [0x%x]!\n", nRet);
    }


    //1.打开数字增益使能
    nRet = MV_CC_SetBoolValue(handle, "GammaEnable", true);
    if (MV_OK != nRet) {
        printf("Set GammaEnable fail! nRet [0x%x]\n", nRet);
    }
    //2.设置gamma类型，user：1，sRGB：2
    nRet = MV_CC_SetEnumValue(handle, "GammaSelector", 1);
    if (MV_OK != nRet) {
        printf("Set GammaSelector fail! nRet [0x%x]\n", nRet);
    }
    //3.设置gamma值，推荐范围0.5-2，1为线性拉伸
    nRet = MV_CC_SetFloatValue(handle, "Gamma", 0.8);
    if (MV_OK != nRet) {
        printf("Set Gamma failed! nRet [0x%x]\n", nRet);
    }


    //开启自动白平衡
    nRet = MV_CC_SetEnumValue(handle, "BalanceWhiteAuto", 1);
    if (MV_OK != nRet) {
        printf("Set BalanceWhiteAuto fail! nRet [0x%x]\n", nRet);
    }


    // 关闭触发模式
    nRet = MV_CC_SetEnumValue(handle, "TriggerMode", 0);
    if (MV_OK != nRet) {
        printf("Set TriggerMode fail! nRet [0x%x]\n", nRet);
    }

    nRet = MV_CC_SetEnumValue(handle, "PixelFormat", PixelType_Gvsp_RGB8_Packed); // 80fps
    //        nRet = MV_CC_SetEnumValue(handle, "PixelFormat", PixelType_Gvsp_BayerRG8); // 160+ fps
    if (MV_OK != nRet) {
        printf("Set PixelFormat fail! nRet [0x%x]\n", nRet);
    }

    nRet = MV_CC_RegisterImageCallBackForRGB(handle, _call_back_ptr, _hik_camera_node);
    if (nRet != MV_OK) {
        printf("register failed! nRet [0x%x]\n", nRet);
    }

}
void HK_Camera::startCamera() { // 需要保证主程序运行
    if (handle == nullptr) {
        std::cout << "handel is null" << std::endl;

    }

    nRet= MV_CC_StartGrabbing(handle);
    if (nRet != MV_OK) {
        printf("Start grabbing failed! nRet [0x%x]\n", nRet);
        return;
    }
    _is_open = true;
    //取流之后，自动白平衡采集一段时间，相机自动调整
    //调整完毕后后，关闭自动白平衡
    for (int i = 3; i > 0; i--) {
        std::cout << "请稍后，正在配置白平衡。(剩余 " << i << " 秒)" << std::endl;
        sleep(1);
    }
    //关闭自动白平衡
    nRet = MV_CC_SetEnumValue(handle, "BalanceWhiteAuto", 0);
    if (MV_OK != nRet) {
        printf("Set BalanceWhiteAuto  fail! nRet [0x%x]\n", nRet);
    } else {
        std::cout << "白平衡配置完成" << std::endl;
    }

}

