#include <armor_tracker_node.h>

namespace armor_auto_aim {

ArmorTrackerNode::ArmorTrackerNode(const rclcpp::NodeOptions &options) : Node("armor_tracker_node", options) {
    declareParameters();
    m_armors_sub = this->create_subscription<armor_interfaces::msg::Armors>("armor_detect", 20, std::bind(&ArmorTrackerNode::subarmorCallback, this, std::placeholders::_1));
}

void ArmorTrackerNode::declareParameters() { 
    
}

void ArmorTrackerNode::subarmorCallback(const armor_interfaces::msg::Armors &armors) {
    

}

} // namespace armor_auto_aim