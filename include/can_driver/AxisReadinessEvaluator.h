#ifndef CAN_DRIVER_AXIS_READINESS_EVALUATOR_H
#define CAN_DRIVER_AXIS_READINESS_EVALUATOR_H

#include "can_driver/SharedDriverState.h"

#include <algorithm>
#include <cstdint>
#include <string>

namespace can_driver {

enum class AxisReadinessPhase : std::uint8_t {
    Offline = 0,
    Degraded,
    Disabled,
    Enabled,
    Commanding,
    FaultActive,
    Recovering,
};

inline const char *AxisReadinessPhaseName(AxisReadinessPhase phase)
{
    switch (phase) {
    case AxisReadinessPhase::Offline:
        return "Offline";
    case AxisReadinessPhase::Degraded:
        return "Degraded";
    case AxisReadinessPhase::Disabled:
        return "Disabled";
    case AxisReadinessPhase::Enabled:
        return "Enabled";
    case AxisReadinessPhase::Commanding:
        return "Commanding";
    case AxisReadinessPhase::FaultActive:
        return "FaultActive";
    case AxisReadinessPhase::Recovering:
        return "Recovering";
    }
    return "Unknown";
}

struct AxisReadiness {
    SharedDriverState::AxisKey key;
    AxisReadinessPhase phase{AxisReadinessPhase::Offline};
    AxisIntent intent{AxisIntent::None};
    bool deviceReady{false};
    bool feedbackSeen{false};
    bool feedbackFresh{false};
    bool degraded{false};
    bool fault{false};
    bool faultCleared{false};
    bool enabled{false};
    bool enabledReady{false};
    bool commandValid{false};
    bool modeExpected{false};
    bool modeMatched{true};
    bool modeReady{true};
    bool feedbackReady{false};
    bool axisReadyForEnable{false};
    bool axisReadyForRun{false};
    bool recoverConfirmed{false};
};

class AxisReadinessEvaluator {
public:
    struct Config {
        std::int64_t feedbackFreshnessTimeoutNs{500000000LL};
        std::uint32_t recoverConfirmCycles{2};
    };

    AxisReadinessEvaluator() = default;
    explicit AxisReadinessEvaluator(Config config)
        : config_(config)
    {
    }

    AxisReadiness Evaluate(const SharedDriverState::AxisFeedbackState &feedback,
                           const SharedDriverState::AxisCommandState *command,
                           AxisIntent intent,
                           const SharedDriverState::DeviceHealthState *deviceHealth,
                           std::int64_t nowNs = SharedDriverSteadyNowNs())
    {
        AxisReadiness readiness =
            EvaluateBase(feedback, command, intent, deviceHealth, nowNs);
        const bool healthyRecoverSample = readiness.axisReadyForEnable;
        if (intent == AxisIntent::Recover) {
            if (healthyRecoverSample) {
                if (feedback.lastRxSteadyNs != lastRecoverSampleNs_) {
                    lastRecoverSampleNs_ = feedback.lastRxSteadyNs;
                    if (lastReadiness_.intent == AxisIntent::Recover &&
                        lastReadiness_.axisReadyForEnable) {
                        ++recoverHealthyCycles_;
                    } else {
                        recoverHealthyCycles_ = 1;
                    }
                } else {
                    recoverHealthyCycles_ = std::max<std::uint32_t>(recoverHealthyCycles_, 1u);
                }
                if (recoverHealthyCycles_ < config_.recoverConfirmCycles) {
                    readiness.phase = AxisReadinessPhase::Recovering;
                } else {
                    readiness.recoverConfirmed = true;
                }
            } else {
                recoverHealthyCycles_ = 0;
                lastRecoverSampleNs_ = 0;
                readiness.phase = AxisReadinessPhase::Recovering;
                readiness.recoverConfirmed = false;
            }
        } else {
            recoverHealthyCycles_ = 0;
            lastRecoverSampleNs_ = 0;
            readiness.recoverConfirmed = readiness.axisReadyForEnable;
        }

        lastReadiness_ = readiness;
        return readiness;
    }

    const AxisReadiness &lastReadiness() const
    {
        return lastReadiness_;
    }

    const AxisReadiness &lastStatus() const
    {
        return lastReadiness_;
    }

    static bool ReadyForEnable(const AxisReadiness &readiness)
    {
        return readiness.axisReadyForEnable;
    }

    static bool ReadyForRun(const AxisReadiness &readiness)
    {
        return readiness.axisReadyForRun;
    }

    static bool RecoverConfirmed(const AxisReadiness &readiness)
    {
        return readiness.recoverConfirmed;
    }

    static bool MotionReady(const AxisReadiness &readiness)
    {
        return ReadyForRun(readiness);
    }

    static std::string DescribeBlockReason(const AxisReadiness &readiness)
    {
        if (!readiness.deviceReady || readiness.phase == AxisReadinessPhase::Offline) {
            return "Feedback offline.";
        }
        if (!readiness.feedbackReady) {
            return "Feedback degraded.";
        }
        if (!readiness.faultCleared) {
            return "Fault still active.";
        }
        if (!readiness.enabledReady) {
            return "Axis not enabled.";
        }
        if (!readiness.modeReady) {
            return "Mode not ready.";
        }
        if (readiness.phase == AxisReadinessPhase::Recovering) {
            return "Axis still recovering.";
        }
        return std::string();
    }

    static std::string DescribeMotionBlock(const AxisReadiness &readiness)
    {
        return DescribeBlockReason(readiness);
    }

private:
    AxisReadiness EvaluateBase(const SharedDriverState::AxisFeedbackState &feedback,
                               const SharedDriverState::AxisCommandState *command,
                               AxisIntent intent,
                               const SharedDriverState::DeviceHealthState *deviceHealth,
                               std::int64_t nowNs) const
    {
        AxisReadiness readiness;
        readiness.key = feedback.key;
        readiness.intent = intent;
        readiness.deviceReady = (deviceHealth == nullptr) ? true : deviceHealth->transportReady;
        readiness.feedbackSeen = feedback.feedbackSeen;
        readiness.degraded = feedback.degraded || feedback.consecutiveTimeoutCount > 0;
        readiness.fault = feedback.fault;
        readiness.enabled = feedback.enabled;
        readiness.commandValid = command != nullptr && command->valid;
        readiness.modeExpected = command != nullptr && command->desiredModeValid;

        readiness.feedbackFresh = readiness.feedbackSeen && feedback.lastRxSteadyNs > 0;
        if (readiness.feedbackFresh && config_.feedbackFreshnessTimeoutNs > 0 &&
            nowNs > feedback.lastRxSteadyNs) {
            readiness.feedbackFresh =
                (nowNs - feedback.lastRxSteadyNs) <= config_.feedbackFreshnessTimeoutNs;
        }

        if (readiness.modeExpected) {
            const bool timestampMatched =
                feedback.feedbackSeen && feedback.lastRxSteadyNs > 0 &&
                feedback.lastModeMatchSteadyNs >= feedback.lastRxSteadyNs;
            const bool valueMatched =
                feedback.feedbackSeen && command != nullptr &&
                feedback.mode == command->desiredMode;
            readiness.modeMatched = timestampMatched || valueMatched;
        }

        readiness.feedbackReady = readiness.deviceReady && readiness.feedbackSeen &&
                                  readiness.feedbackFresh && !readiness.degraded;
        readiness.faultCleared = !readiness.fault;
        readiness.enabledReady = readiness.enabled;
        readiness.modeReady = !readiness.modeExpected || readiness.modeMatched;
        readiness.axisReadyForEnable = readiness.feedbackReady && readiness.faultCleared;
        readiness.axisReadyForRun =
            readiness.axisReadyForEnable && readiness.enabledReady && readiness.modeReady;

        if (!readiness.deviceReady || !readiness.feedbackSeen) {
            readiness.phase = AxisReadinessPhase::Offline;
            return readiness;
        }
        if (!readiness.feedbackReady) {
            readiness.phase = AxisReadinessPhase::Degraded;
            return readiness;
        }
        if (!readiness.faultCleared) {
            readiness.phase = AxisReadinessPhase::FaultActive;
            return readiness;
        }
        if (!readiness.enabledReady) {
            readiness.phase = AxisReadinessPhase::Disabled;
            return readiness;
        }

        readiness.phase =
            readiness.commandValid ? AxisReadinessPhase::Commanding : AxisReadinessPhase::Enabled;
        return readiness;
    }

    Config config_{};
    AxisReadiness lastReadiness_{};
    std::uint32_t recoverHealthyCycles_{0};
    std::int64_t lastRecoverSampleNs_{0};
};

} // namespace can_driver

#endif // CAN_DRIVER_AXIS_READINESS_EVALUATOR_H
