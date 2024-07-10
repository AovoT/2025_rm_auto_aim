#ifndef ARMOR_DETECT_H
#define ARMOR_DETECT_H

#include <iostream>
#include <vector>

#include <eigen3/Eigen/Core>

#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>


namespace armor_auto_aim {
struct AllThresold {
    //处理方式
    std::string process_way; // hsv or rgb;
    //侦查颜色
    std::string detect_color;
    // hsv预处理阈值
    int h_up;
    int s_up;
    int v_up;
    int h_lower;
    int s_lower;
    int v_lower;
    // rgb预处理阈值
    int gray_thresold;
    int rgb_thresold;

    int morphologyex_times; // 膨胀次数

    // 筛选装甲板||灯条的一系列阈值
    int min_light_area; // 灯条最小面积
    int max_light_area; // 灯条最大面积
    int max_light_angle; // 灯条与水平线最大的角度
    float min_light_lenght_width_ratio; // 灯条最小的长宽比;
    float max_light_lenght_width_ratio; // 灯条最大的长宽比;

    float min_armor_light_area_ratio;  // 装甲板两个灯条最小面积比
    float max_armor_light_area_ratio; // 装甲板两灯条最大面积比
    float min_armor_lenght_width_ratio; // 装甲板最小的长宽比;
    float max_armor_lenght_width_ratio; // 装甲板最大的长宽比;
    int min_armor_area; // 装甲板最小面积
    int max_armor_area; // 装甲板最大面积

    // 装甲板数字分类的阈值
    float confidence;
};

enum ArmorType {
    SMALL, BIG
};

struct Light {
    cv::RotatedRect rotated_rect;
    cv::Point2f center;
    std::vector<cv::Point2f> points; // 依次为上 下
    float length;
    float width;
    float angle; // 与水平线的夹角
    float lenght_width_ratio;
    float area;

};

struct Armor {
    bool ifsmall;
    Light left_light;
    Light right_light;
    int number_class;
    std::vector<cv::Point2f> points; // 依次为  左上  左下  右下  右上
    cv::RotatedRect rotated_rect;
    cv::Point2f center;
    float length;
    float width;
    float angle;
    float area;
    float lenght_width_ratio;
    cv::Mat number_image = cv::Mat();
};

class ArmorDetect {
public:
    ArmorDetect(const AllThresold &all_thresold);
    ArmorDetect() = default;
    // 整体流程
    void startDetect(const cv::Mat &image);
    cv::Mat preprocessImage(const cv::Mat &image);  // 图像预处理
    std::vector<Light> getRelLights(cv::Mat const &processed_image); // 获取真正的灯条
    void matchRelArmor(const std::vector<Light> &lights); //获取装甲板(未进过数字分类)
    void armorClassifySmallLarge();
    void cropArmorNumber(const cv::Mat &image); //  裁取标准装甲板
    void inferenceArmorNumber();
    std::vector<Armor> getArmors(); // 获取装甲板
    void drawArmor(const cv::Mat &image);
    void updateArmors(const std::vector<Armor> &armors);
    
    //更新参数函数
    void updateAllThresold(const AllThresold &all_thresold);
private:
    //  辅助函数
    bool judgeIfRelLight(const Light &demo_light); // 判断是否为真正的灯条
    bool judgeIfRelArmor(const Armor &armor); // 判断是否为装甲板
    Light processLight(const cv::RotatedRect & rotated_rect); // 生成灯条
    Armor processArmor(const Light &left_light, const Light &right_light); // 生成装甲板

    AllThresold m_all_thresold;
    std::vector<Armor> m_armors; 
};
}

#endif  // ARMOR_DETECT_H