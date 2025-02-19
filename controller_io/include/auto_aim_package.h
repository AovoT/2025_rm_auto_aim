#ifndef AUTO_AIM_PACKAGE_H
#define AUTO_AIM_PACKAGE_H

#include <cstdint>

namespace armor_auto_aim {
struct GimbalPosePacket {
    uint64_t is_outpost;
    float w, x, y, z;
    uint16_t delay;
    float px, py, pz;
};
struct AutoAimPacket {
    float x, y, z,
          v_x, v_y, v_z,
          theta, omega,
          r1, r2, dz;
    uint8_t num;
    uint8_t id;
    uint8_t delay;
    uint8_t is_tracking;

};
} // armor_auto_aim




#endif // AUTO_AIM_PACKAGE_N