#include <armor_detect.h>

namespace armor_auto_aim {
ArmorDetect::ArmorDetect(const AllThresold &all_thresold) : m_all_thresold(all_thresold) {}

// ArmorDetect::~ArmorDetect() = default;


cv::Mat ArmorDetect::preprocessImage(const cv::Mat &image) {
    // rgb和hsv两种处理方式
    cv::Mat processed_image;
    if (m_all_thresold.process_way == "gray") {
        cv::cvtColor(image, processed_image, cv::COLOR_BGR2GRAY);
        cv::threshold(processed_image, processed_image, m_all_thresold.gray_thresold, 255, 0);
        cv::Mat kernel = cv::getStructuringElement(2, cv::Size(3, 3));
        cv::morphologyEx(processed_image, processed_image ,1,kernel,cv::Point(-1,-1),m_all_thresold.morphologyex_times);
    }
    if (m_all_thresold.process_way == "rgb") {
        cv::Mat gray_image, rgb_image;
        std::vector<cv::Mat> split_image;
        cv:split(image, split_image);
        // 通道减保留颜色特征
        if (m_all_thresold.detect_color == "BLUE") {
            rgb_image = split_image[0] - split_image[2];

        } else {
            rgb_image = split_image[2] - split_image[0];
        }
        cv::threshold(rgb_image, rgb_image, m_all_thresold.rgb_thresold, 255, 0);
        cv::cvtColor(image, gray_image, 10);
        //保留亮度特征
        cv::threshold(gray_image, gray_image, m_all_thresold.gray_thresold, 255, 0);
        //融合两个特征
        cv::bitwise_and(gray_image, rgb_image, processed_image);
        // 形态学操作
        cv::Mat kernel = cv::getStructuringElement(2, cv::Size(3, 3));
        cv::morphologyEx(processed_image, processed_image ,1,kernel,cv::Point(-1,-1),m_all_thresold.morphologyex_times);
    } else if (m_all_thresold.process_way == "hsv") {
        cv::Mat hsv_image;
        // hsv阈值处理
        cv::cvtColor(image,hsv_image, cv::COLOR_BGR2HSV);   
        cv::inRange(hsv_image, cv::Scalar(m_all_thresold.h_lower, m_all_thresold.s_lower, m_all_thresold.v_lower), 
                               cv::Scalar(m_all_thresold.h_up, m_all_thresold.s_up, m_all_thresold.v_up),  processed_image);
        // 形态学操作                                
        cv::Mat kernel = cv::getStructuringElement(2, cv::Size(3, 3));
        cv::morphologyEx(processed_image, processed_image ,1 ,kernel,cv::Point(-1,-1), m_all_thresold.morphologyex_times);
    }
    cv::imshow("pre_process_image", processed_image);
    cv::waitKey(1);
    return processed_image;    
}

std::vector<Light> ArmorDetect::getRelLights(cv::Mat const & processed_image) {
    // 寻找轮廓
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(processed_image, contours, hierarchy, 0, 2);
    cv::RotatedRect demo_light;
    std::vector<Light> lights;
    // 筛选轮廓
    for (const auto & contour : contours) {
        if (contour.size() < 6) {
            continue;
        } else if (cv::contourArea(contour) < m_all_thresold.min_light_area) {
            continue;
        } else if (cv::contourArea(contour) > m_all_thresold.max_light_area) {
            continue;
        }
        auto rotated_rect = cv::fitEllipse(contour);
        Light demo_light = processLight(rotated_rect);

        if (judgeIfRelLight(demo_light)) {
            lights.push_back(demo_light);
        }
    }
    // 将所有的灯条按照从左到右排列起来
    if (lights.size() >= 2) {
        for (int i = 0; i < lights.size(); i++) {
            for (int j = i+1; j < lights.size(); j++) {
                Light temporay_light;
                if (lights[i].center.x > lights[j].center.x) {
                    temporay_light = lights[j];
                    lights[j] = lights[i];
                    lights[i] = temporay_light;
                }
            }
        }
    }
    return lights;
}

void ArmorDetect::matchRelArmor(const std::vector<Light> &lights) {
    for (int i = 0; i < lights.size(); i++) {
        for (int j = i+1; j < lights.size(); j++) {
            Armor demo_armor = processArmor(lights[i], lights[j]);
            if (judgeIfRelArmor(demo_armor)) {
                m_armors.push_back(demo_armor);
            }
        }
    }
}

bool ArmorDetect::judgeIfRelLight(const Light &demo_light) {
    return demo_light.angle < m_all_thresold.max_light_angle &&
           demo_light.lenght_width_ratio  < m_all_thresold.max_light_lenght_width_ratio &&
           demo_light.lenght_width_ratio  > m_all_thresold.min_light_lenght_width_ratio;
}

bool ArmorDetect::judgeIfRelArmor(const Armor &armor) {
    return armor.lenght_width_ratio < m_all_thresold.max_armor_lenght_width_ratio &&
           armor.lenght_width_ratio > m_all_thresold.min_armor_lenght_width_ratio &&
           armor.left_light.area / armor.right_light.area < m_all_thresold.max_armor_light_area_ratio &&
           armor.left_light.area / armor.right_light.area > m_all_thresold.min_armor_light_area_ratio &&
           armor.area < m_all_thresold.max_armor_area &&
           armor.area > m_all_thresold.min_armor_area;

}

void ArmorDetect::updateAllThresold(const AllThresold &all_thresold) {
    m_all_thresold = all_thresold;
}

Light ArmorDetect::processLight(const cv::RotatedRect &rotated_rect) {
    Light light;
    light.center = rotated_rect.center;
    if (rotated_rect.angle>90) {
        light.angle = rotated_rect.angle - 180;
    } else {
        light.angle = rotated_rect.angle;
    }
    std::vector<cv::Point2f> points(4);
    rotated_rect.points(points.data());    
    light.length = std::min(rotated_rect.size.height,rotated_rect.size.width);
    light.width = std::max(rotated_rect.size.height,rotated_rect.size.width);
    light.lenght_width_ratio = light.length / light.width;
    for (int i = 0; i < 4; i++) {
        for (int j = i + 1; j < 4; j++) {
            if (points[i].y > points[j].y) {
                cv::Point2f temp = points[i];
                points[i] = points[j];
                points[j] = temp;
            }
        }
    }
    light.points.push_back(cv::Point2f((points[0] + points[1]) / 2));
    light.points.push_back(cv::Point2f((points[2] + points[3]) / 2));
    light.area = rotated_rect.size.area();
    return light;
}

Armor ArmorDetect::processArmor(const Light &left_light, const Light &right_light) {
    Armor armor;
    armor.left_light = left_light;
    armor.right_light = right_light;
    armor.center = (left_light.center + right_light.center) / 2;
    armor.angle = (left_light.angle + right_light.angle) / 2;
    armor.width = (left_light.width + right_light.width) / 2;
    armor.length = std::sqrt((right_light.center.x - left_light.center.x) * (right_light.center.x - left_light.center.x) + (right_light.center.y - left_light.center.y) * (right_light.center.y - left_light.center.y));
    armor.area = armor.length * armor.width;
    armor.lenght_width_ratio = armor.length / armor.width;
    armor.points.push_back(left_light.points[0]);
    armor.points.push_back(left_light.points[1]);
    armor.points.push_back(right_light.points[1]);
    armor.points.push_back(right_light.points[0]);
    return armor;
}

void ArmorDetect::startDetect(const cv::Mat &image) {
    m_armors.clear();
    m_armors = std::vector<Armor>();
    cv::Mat processed_image = preprocessImage(image);
    std::vector<Light> lights = getRelLights(processed_image);
    matchRelArmor(lights);
    armorClassifySmallLarge();
    cropArmorNumber(image);
    // drawArmor(image);
    // m_armors.clear();
}

void ArmorDetect::drawArmor(const cv::Mat &image) {
    cv::Mat drawed_image = image.clone();  // 深拷贝图像，以免修改原图像
    for (const auto &armor : m_armors) {
        // 画出装甲板的边框
        cv::line(drawed_image, armor.points[0], armor.points[1], cv::Scalar(0, 0, 255), 2);
        cv::line(drawed_image, armor.points[1], armor.points[2], cv::Scalar(0, 0, 255), 2);
        cv::line(drawed_image, armor.points[2], armor.points[3], cv::Scalar(0, 0, 255), 2);
        cv::line(drawed_image, armor.points[3], armor.points[0], cv::Scalar(0, 0, 255), 2);

        // 标记每个角点的编号，增加不同的偏移量
        for (size_t i = 0; i < armor.points.size(); ++i) {
            cv::circle(drawed_image, armor.points[i], 5, cv::Scalar(0, 255, 0), -1); // 画点

            // 根据点的位置偏移文字，避免重叠
            cv::Point2f offset;
            switch (i) {
                case 0: offset = cv::Point2f(-15, -10); break; // 左上
                case 1: offset = cv::Point2f(-15, 15); break;  // 左下
                case 2: offset = cv::Point2f(10, 15); break;   // 右下
                case 3: offset = cv::Point2f(10, -10); break;  // 右上
            }
            cv::putText(drawed_image, std::to_string(i), armor.points[i] + offset,
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 1);  // 标记编号
        }

        // 标记中心点和左右灯条
        cv::circle(drawed_image, armor.center, 10, cv::Scalar(0, 255, 255), 1, 8, 0);
        cv::putText(drawed_image, "Center", armor.center + cv::Point2f(-10, 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 1);

        cv::circle(drawed_image, armor.left_light.center, 10, cv::Scalar(255, 255, 0), 1, 8, 0);
        cv::putText(drawed_image, "Left", armor.left_light.center + cv::Point2f(-10, 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 1);

        cv::circle(drawed_image, armor.right_light.center, 10, cv::Scalar(255, 255, 0), 1, 8, 0);
        cv::putText(drawed_image, "Right", armor.right_light.center + cv::Point2f(-10, 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 1);
    }

    // 显示图像
    cv::imshow("drawed_armor_image", drawed_image);
    cv::waitKey(1);
}

void ArmorDetect::cropArmorNumber(const cv::Mat &image) {
    
    if (image.empty() || m_armors.empty()) {
        return;
    }

    const int croped_image_light_y_top = 7;
    const int croped_image_light_y_bottom = 19;
    const int croped_image_height = 28;
    const int croped_small_image_width = 32;
    const int croped_large_image_width = 54;
    const cv::Size roi_size(20, 28);

    for (auto &armor : m_armors) {
        const int croped_image_width = armor.ifsmall ? croped_small_image_width : croped_large_image_width;
        cv::Point2f armor_points[4] = {
            armor.points[0],
            armor.points[1],
            armor.points[2],
            armor.points[3]
        };
        cv::Point2f target_points[4] = {
            cv::Point2f(0, croped_image_light_y_top),
            cv::Point2f(0, croped_image_light_y_bottom),
            cv::Point2f(croped_image_width - 1, croped_image_light_y_bottom),
            cv::Point2f(croped_image_width - 1, croped_image_light_y_top)
        }
;
        cv::Mat number_image;
        auto rotation_matrix = cv::getPerspectiveTransform(armor_points, target_points);
        cv::warpPerspective(image, number_image, rotation_matrix, cv::Size(croped_image_width, croped_image_height));
        if (number_image.cols < roi_size.width || number_image.rows < roi_size.height) {
             std::cerr << "Error: ROI size is out of the number_image bounds." << std::endl;
            return;
        }

        number_image = number_image(cv::Rect(cv::Point((croped_image_width - roi_size.width) / 2, 0), roi_size));

         // Binarize
        cv::cvtColor(number_image, number_image, cv::COLOR_RGB2GRAY);
        cv::threshold(number_image, number_image, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
        armor.number_image = number_image;
        // number_image.reshape(460, 460);
        cv::imshow("number_image", number_image);
        cv::waitKey(1);
    }
}

void ArmorDetect::updateArmors(const std::vector<Armor> &armors) {
    m_armors = armors;
}

std::vector<Armor> ArmorDetect::getArmors() {
    if (!m_armors.empty()) {
        return m_armors;
    }

    return std::vector<Armor>();
}

void ArmorDetect::armorClassifySmallLarge() {
    for (auto &armor : m_armors) {
        if (armor.lenght_width_ratio > 1.4) {
            armor.ifsmall = false;
        } else {
            armor.ifsmall = true;
        }
    }
}
} // namespace armor_auto_aim
