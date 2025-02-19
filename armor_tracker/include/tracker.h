#ifndef ARMOR_TRACKER_TRACK_H
#define ARMOR_TRACKER_TRACK_H

#include <map>
#include <memory>

#include <geometry_msgs/msg/quaternion.hpp>
#include <angles/angles.h>


#include "kalman_filter.h"
#include <armor_interfaces/msg/armor.hpp>
#include <armor_interfaces/msg/armors.hpp>
#include <armor_interfaces/msg/target.hpp>

namespace armor_auto_aim {
struct Armor {
    struct Pose {
        float x;
        float y;
        float z;
    };
    Pose pose;
    float yaw;
};

struct AngleInfo {
    double yaw = 0 ;
    double pitch = 0;
    double roll = 0;
    double yaw_angle = 0 ;
    double pitch_angle = 0;
    double roll_angle = 0;
};
class Tracker;


class TrackerStateMachine {
public:
    enum class State {
        Lost,
        Detecting,
        Tracking,
        TempLost
    };

    TrackerStateMachine() =default;

    inline void initState() { 
        std::cout << "init state" << std::endl;
        m_state = State::Detecting;
         }

    [[nodiscard]] inline State state() const { return m_state; };

    [[nodiscard]] inline std::string stateString() const { return m_state_map.find(m_state)->second; }

    void update(bool detector_result);

    void setTrackingCount(int v) { m_tracking_threshold = v; }

    void setlostcount(int v) { m_lost_threshold = v; }
private:
    State m_state = State::Lost;
    int m_detect_count = 0;
    int m_lost_count = 0;
    int m_tracking_threshold = 0;
    int m_lost_threshold = 0;
    std::map<State, std::string> m_state_map {
            { State::Lost, "Lost" },
            { State::Detecting, "Detecting" },
            { State::Tracking, "Tracking" },
            { State::TempLost, "TempLost" }
    };
};

class Tracker {
public:
    Armor getLeftArmor() {return m_left_armor;}
    Armor getRightArmor() {return m_right_armor;}
    Tracker() =default;

    Armor computeOtherArmor(const Eigen::Vector3d& center, double radius, double base_yaw, double angle_offset);
    Eigen::Vector3d computeRotationCenter(const Armor& armor, double radius);
    void initTracker(const armor_interfaces::msg::Armors::SharedPtr armors_msg);

    void updateTracker(const armor_interfaces::msg::Armors::SharedPtr armors_msg);

    [[nodiscard]] inline TrackerStateMachine::State state() const { return m_tracker_state_machine.state(); }

    [[nodiscard]] inline std::string stateString() const { return m_tracker_state_machine.stateString(); }

    inline bool isTracking() const {
        return m_tracker_state_machine.state() == TrackerStateMachine::State::Tracking;
    }

    double getLastYaw() const { return m_last_yaw; }

    int getNum() const { return m_armor_num; }



    void setMatchDistance(double v) { m_max_match_distance = v; }

    void setMatchYaw(double v) { m_max_match_yaw = v; }

    void setComlexPatternMode(int v) { m_comlex_pattern_mode = v; }

    TrackerStateMachine* getStateMachine() { return &m_tracker_state_machine; }

    [[nodiscard]] const Eigen::VectorXd& getTargetPredictSate() const { return m_target_predict_state; }

    std::shared_ptr<ExtendedKalmanFilter> ekf = nullptr;
    armor_interfaces::msg::Armor tracked_armor;
    Eigen::VectorXd measurement;
    double dz, another_r;
private:
    double m_max_match_distance = 2.0;
    double m_max_match_yaw = 1.0;
    Eigen::VectorXd m_target_predict_state;
    TrackerStateMachine m_tracker_state_machine;
    int m_tracked_id{};  // armor number
    double m_last_yaw = 0.0;
    double m_last_pitch = 0.0;
    Armor m_left_armor;
    Armor m_right_armor;   double m_last_roll = 0.0;


    int m_armor_num = 0;
    // complex pattern
    bool m_is_complex_pattern = false;
    int m_comlex_pattern_mode = 0; // 0: off; 1: outpost; 2: sentry;

    void initEkf(const armor_interfaces::msg::Armor& armor);

    void handleArmorJump(const armor_interfaces::msg::Armor& same_id_armor);

    void updateArmorNum(const armor_interfaces::msg::Armor& armor);

    double orientationToYaw(const geometry_msgs::msg::Quaternion& q);
    AngleInfo orientationToAngle(const geometry_msgs::msg::Quaternion& q);

    Eigen::Vector3d getArmorPositionFromState(const Eigen::VectorXd& x);

    static double normalize_radians(double rad) {
        double pi = std::atan(1) * 4;
        double result = std::fmod(rad, 2 * pi);
        if (result <= -pi)
            result += 2 * pi;
        else if (result > pi)
            result -= 2 * pi;
        return result;
    }
};
} // armor_auto_aim

#endif //ARMOR_TRACKER_TRACK_H
