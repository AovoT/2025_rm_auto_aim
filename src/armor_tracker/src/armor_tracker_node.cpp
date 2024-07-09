#include <armor_tracker_node.h>

namespace armor_auto_aim {

ArmorTrackerNode::ArmorTrackerNode(const rclcpp::NodeOptions &options) : Node("armor_tracker_node", options) {
    declareParameters();
    
    
}

void ArmorTrackerNode::declareParameters() { 
    
}

} // namespace armor_auto_aim