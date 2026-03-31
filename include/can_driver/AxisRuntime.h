#ifndef CAN_DRIVER_AXIS_RUNTIME_H
#define CAN_DRIVER_AXIS_RUNTIME_H

#include "can_driver/AxisReadinessEvaluator.h"

namespace can_driver {

using AxisRuntimeState = AxisReadinessPhase;
using AxisRuntimeStatus = AxisReadiness;
using AxisRuntime = AxisReadinessEvaluator;

inline const char *AxisRuntimeStateName(AxisRuntimeState state)
{
    return AxisReadinessPhaseName(state);
}

} // namespace can_driver

#endif // CAN_DRIVER_AXIS_RUNTIME_H
