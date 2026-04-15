#ifndef CAN_DRIVER_EYOU_PH_CAN_H
#define CAN_DRIVER_EYOU_PH_CAN_H

#include "can_driver/CanProtocol.h"
#include "can_driver/CanTransport.h"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

/**
 * @brief 摆臂 PH 电机协议（CANopen SDO）实现。
 *
 * 说明：
 * - 命令通过 `CanTransport` 发送标准 8 字节“CAN 语义帧”；
 * - 可与 `UdpCanTransport` 组合，实现“UDP 通道上的 CANopen 控制”；
 * - 也可在直连 SocketCAN 场景复用。
 */
class EyouPhCan : public CanProtocol {
public:
    explicit EyouPhCan(std::shared_ptr<CanTransport> controller);
    ~EyouPhCan() override;

    bool setMode(MotorID motorId, MotorMode mode) override;
    bool setVelocity(MotorID motorId, int32_t velocity) override;
    bool setAcceleration(MotorID motorId, int32_t acceleration) override;
    bool setDeceleration(MotorID motorId, int32_t deceleration) override;
    bool setPosition(MotorID motorId, int32_t position) override;

    bool Enable(MotorID motorId) override;
    bool Disable(MotorID motorId) override;
    bool Stop(MotorID motorId) override;

    int64_t getPosition(MotorID motorId) const override;
    int16_t getCurrent(MotorID motorId) const override;
    int16_t getVelocity(MotorID motorId) const override;

    bool isEnabled(MotorID motorId) const override;
    bool hasFault(MotorID motorId) const override;

    void initializeMotorRefresh(const std::vector<MotorID> &motorIds) override;
    void setRefreshRateHz(double hz);

private:
    struct MotorState {
        int32_t position = 0;
        int16_t current = 0;
        int16_t velocity = 0;
        bool positionReceived = false;
        bool currentReceived = false;
        bool velocityReceived = false;
        bool enabled = false;
        bool fault = false;
        MotorMode mode = MotorMode::Velocity;
    };

    std::shared_ptr<CanTransport> canController;
    mutable std::mutex stateMutex;
    mutable std::unordered_map<uint16_t, MotorState> motorStates;

    std::size_t receiveHandlerId = 0;

    std::vector<uint16_t> refreshMotorIds;
    mutable std::unordered_set<uint16_t> managedMotorIds;
    mutable std::mutex refreshMutex;
    std::atomic<bool> refreshLoopActive{false};
    std::thread refreshThread;
    std::atomic<double> refreshRateHz_{0.0};

    void registerManagedMotorId(uint16_t motorId) const;
    bool isManagedMotorId(uint16_t motorId) const;

    void sendSdoWriteU8(uint16_t canId, uint16_t index, uint8_t subIndex, uint8_t value) const;
    void sendSdoWriteU16(uint16_t canId, uint16_t index, uint8_t subIndex, uint16_t value) const;
    void sendSdoWriteI32(uint16_t canId, uint16_t index, uint8_t subIndex, int32_t value) const;
    void sendSdoRead(uint16_t canId, uint16_t index, uint8_t subIndex) const;

    void requestPosition(uint16_t canId) const;
    void requestCurrent(uint16_t canId) const;
    void requestVelocity(uint16_t canId) const;
    void requestStatusWord(uint16_t canId) const;

    void refreshMotorStates();
    void stopRefreshLoop();
    std::chrono::milliseconds computeRefreshSleep(std::size_t motorCount) const;
    void handleResponse(const CanTransport::Frame &frame);
};

#endif // CAN_DRIVER_EYOU_PH_CAN_H
