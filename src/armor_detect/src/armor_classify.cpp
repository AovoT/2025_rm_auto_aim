#include <armor_classify.h>

namespace armor_auto_aim {

ArmorNumberClassify::ArmorNumberClassify(const ClassifyInfo &info) {
    
    ov::Core ie;

    m_model = ie.read_model(info.model_path);

    m_network = ie.compile_model(m_model, info.infer_place);

    m_infer_request = m_network.create_infer_request();

    std::cout << "Model inputs:\n";
    for (const auto& input : m_model->inputs()) {
        std::cout << input.get_any_name() << ": " << input.get_shape() << " - " << input.get_element_type() << "\n";
    }
    std::cout << "Model outputs:\n";
    for (const auto& output : m_model->outputs()) {
        std::cout << output.get_any_name() << ": " << output.get_shape() << " - " << output.get_element_type() << "\n";
    }
}

void ArmorNumberClassify::startClassify(std::vector<Armor> &armors) {
    for (auto &armor : armors) {
        if (armor.number_image.empty()) {
            continue;
        }
        cv::Mat wait_infer_image;
        armor.number_image.convertTo(wait_infer_image,CV_32F);
        ov::element::Type type = m_model->input().get_element_type();
        ov::Shape shape = m_model->input().get_shape();
        
        wait_infer_image = wait_infer_image / 255.0;
        ov::Tensor input_tensor(type, shape);
        std::memcpy(input_tensor.data(), wait_infer_image.data, input_tensor.get_byte_size());
        m_infer_request.set_input_tensor(input_tensor);
        m_infer_request.infer();
        handleData(armor);
    }
}

void ArmorNumberClassify::handleData(Armor &armor) {
    ov::Tensor output_tensor = m_infer_request.get_output_tensor();
    auto output_data = output_tensor.data<float>();
    float *end_ptr = output_data + 5;
    float max_prob = *std::max_element(output_data, end_ptr);

    std::vector<float> exp_output(5);
    std::transform(output_data, end_ptr, exp_output.begin(), [max_prob](float val) {
        return exp(val - max_prob);
    });

    float sum = std::accumulate(exp_output.begin(), exp_output.end(), 0.0f);
    std::transform(exp_output.begin(), exp_output.end(), exp_output.begin(), [sum](float val) {
        return val / sum;
    });
    auto max_it = std::max_element(exp_output.begin(), exp_output.end());
    int label_id = std::distance(exp_output.begin(), max_it) + 1;
    double confidence = *max_it;
    // std::cout << confidence << std::endl;
    if (confidence > 0.95 ) {
        // std::cout << "label_id = " << label_id << " " << "confidence = " << confidence << std::endl;
        armor.number_class = label_id;
        
    } else {
        // std::cout << "不是数字" << std::endl;
        armor.number_class = 0;
    }
    


    





    // float max_prob = std::max_element(output_data.begin(), output_data.end());
    // cv::Mat softmax_prob;
    // cv::exp(outputs - max_prob, softmax_prob);
    // float sum = static_cast<float>(cv::sum(softmax_prob)[0]);
    // softmax_prob /= sum;

    // double confidence;
    // cv::Point class_id_point;
    // minMaxLoc(softmax_prob.reshape(1, 1), nullptr, &confidence, nullptr, &class_id_point);
    // std::cout << output_data[1] << std::endl;
}

} // namespace armor_auto_aim