#ifndef CAN_DRIVER_SAFE_COMMAND_H
#define CAN_DRIVER_SAFE_COMMAND_H

#include <cstdint>
#include <string>

namespace can_driver {
namespace safe_command {

int32_t clampToInt32(double value);
int16_t clampToInt16(double value);
bool scaleAndClampToInt32(double cmdValue,
                          double scale,
                          const std::string &jointName,
                          int32_t &rawValueOut);

} // namespace safe_command
} // namespace can_driver

#endif // CAN_DRIVER_SAFE_COMMAND_H
