#ifndef ARMOR_CLASSIFY_H
#define ARMOR_CLASSIFY_H

#include <iostream>
#include <vector>
#include <string>

#include <openvino/openvino.hpp>

#include "armor_detect.h"

namespace armor_auto_aim {
struct ClassifyInfo {
    std::string model_path;
    std::string infer_place;
    float confidence;
};

class ArmorNumberClassify {
public:
    ArmorNumberClassify() = default;

    ArmorNumberClassify(const ClassifyInfo &info);

    void startClassify(std::vector<Armor> &armors);

    void handleData(Armor &armors);
private:
    float m_confidence;
    std::shared_ptr<ov::Model> m_model;
    ov::CompiledModel m_network;
    ov::InferRequest m_infer_request;
};
} // namespace armor_auto_aim
#endif