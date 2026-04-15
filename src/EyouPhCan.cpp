#include "can_driver/EyouPhCan.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace {
constexpr std::size_t kQueriesPerMotorPerCycle = 4;

int16_t readInt16LE(const CanTransport::Frame &frame, std::size_t index)
{
    if (index + 1 >= frame.dlc) {
        return 0;
    }
    const int v = static_cast<int>(frame.data[index]) |
                  (static_cast<int>(frame.data[index + 1]) << 8);
    return static_cast<int16_t>(v);
}

int32_t readInt32LE(const CanTransport::Frame &frame, std::size_t index)
{
    if (index + 3 >= frame.dlc) {
        return 0;
    }
    const uint32_t v = static_cast<uint32_t>(frame.data[index]) |
                       (static_cast<uint32_t>(frame.data[index + 1]) << 8) |
                       (static_cast<uint32_t>(frame.data[index + 2]) << 16) |
                       (static_cast<uint32_t>(frame.data[index + 3]) << 24);
    return static_cast<int32_t>(v);
}

} // namespace

EyouPhCan::EyouPhCan(std::shared_ptr<CanTransport> controller)
    : canController(std::move(controller))
{
    if (canController) {
        receiveHandlerId = canController->addReceiveHandler(
            [this](const CanTransport::Frame &frame) { handleResponse(frame); });
    }
}

EyouPhCan::~EyouPhCan()
{
    stopRefreshLoop();
    if (canController && receiveHandlerId != 0) {
        canController->removeReceiveHandler(receiveHandlerId);
    }
}

std::chrono::milliseconds EyouPhCan::computeRefreshSleep(std::size_t motorCount) const
{
    const double hz = refreshRateHz_.load(std::memory_order_relaxed);
    if (std::isfinite(hz) && hz > 0.0) {
        const auto intervalMs = static_cast<int64_t>(std::llround(1000.0 / hz));
        return std::chrono::milliseconds(std::max<int64_t>(1, intervalMs));
    }
    const std::size_t intervalMs = std::max<std::size_t>(5, motorCount * kQueriesPerMotorPerCycle);
    return std::chrono::milliseconds(intervalMs);
}

void EyouPhCan::setRefreshRateHz(double hz)
{
    if (!std::isfinite(hz) || hz <= 0.0) {
        refreshRateHz_.store(0.0, std::memory_order_relaxed);
        return;
    }
    refreshRateHz_.store(hz, std::memory_order_relaxed);
}

void EyouPhCan::registerManagedMotorId(uint16_t motorId) const
{
    std::lock_guard<std::mutex> lock(refreshMutex);
    managedMotorIds.insert(motorId);
}

bool EyouPhCan::isManagedMotorId(uint16_t motorId) const
{
    std::lock_guard<std::mutex> lock(refreshMutex);
    if (managedMotorIds.empty()) {
        return true;
    }
    return managedMotorIds.find(motorId) != managedMotorIds.end();
}

void EyouPhCan::sendSdoWriteU8(uint16_t canId,
                               uint16_t index,
                               uint8_t subIndex,
                               uint8_t value) const
{
    if (!canController) {
        return;
    }
    CanTransport::Frame frame;
    frame.id = canId;
    frame.isExtended = false;
    frame.isRemoteRequest = false;
    frame.dlc = 8;
    frame.data.fill(0);

    frame.data[0] = 0x2F;
    frame.data[1] = static_cast<uint8_t>(index & 0xFF);
    frame.data[2] = static_cast<uint8_t>((index >> 8) & 0xFF);
    frame.data[3] = subIndex;
    frame.data[4] = value;

    canController->send(frame);
}

void EyouPhCan::sendSdoWriteU16(uint16_t canId,
                                uint16_t index,
                                uint8_t subIndex,
                                uint16_t value) const
{
    if (!canController) {
        return;
    }
    CanTransport::Frame frame;
    frame.id = canId;
    frame.isExtended = false;
    frame.isRemoteRequest = false;
    frame.dlc = 8;
    frame.data.fill(0);

    frame.data[0] = 0x2B;
    frame.data[1] = static_cast<uint8_t>(index & 0xFF);
    frame.data[2] = static_cast<uint8_t>((index >> 8) & 0xFF);
    frame.data[3] = subIndex;
    frame.data[4] = static_cast<uint8_t>(value & 0xFF);
    frame.data[5] = static_cast<uint8_t>((value >> 8) & 0xFF);

    canController->send(frame);
}

void EyouPhCan::sendSdoWriteI32(uint16_t canId,
                                uint16_t index,
                                uint8_t subIndex,
                                int32_t value) const
{
    if (!canController) {
        return;
    }
    CanTransport::Frame frame;
    frame.id = canId;
    frame.isExtended = false;
    frame.isRemoteRequest = false;
    frame.dlc = 8;
    frame.data.fill(0);

    frame.data[0] = 0x23;
    frame.data[1] = static_cast<uint8_t>(index & 0xFF);
    frame.data[2] = static_cast<uint8_t>((index >> 8) & 0xFF);
    frame.data[3] = subIndex;
    frame.data[4] = static_cast<uint8_t>(value & 0xFF);
    frame.data[5] = static_cast<uint8_t>((value >> 8) & 0xFF);
    frame.data[6] = static_cast<uint8_t>((value >> 16) & 0xFF);
    frame.data[7] = static_cast<uint8_t>((value >> 24) & 0xFF);

    canController->send(frame);
}

void EyouPhCan::sendSdoRead(uint16_t canId, uint16_t index, uint8_t subIndex) const
{
    if (!canController) {
        return;
    }
    CanTransport::Frame frame;
    frame.id = canId;
    frame.isExtended = false;
    frame.isRemoteRequest = false;
    frame.dlc = 8;
    frame.data.fill(0);

    frame.data[0] = 0x40;
    frame.data[1] = static_cast<uint8_t>(index & 0xFF);
    frame.data[2] = static_cast<uint8_t>((index >> 8) & 0xFF);
    frame.data[3] = subIndex;

    canController->send(frame);
}

void EyouPhCan::requestPosition(uint16_t canId) const
{
    sendSdoRead(canId, 0x6064, 0x00);
}

void EyouPhCan::requestCurrent(uint16_t canId) const
{
    sendSdoRead(canId, 0x6077, 0x00);
}

void EyouPhCan::requestVelocity(uint16_t canId) const
{
    sendSdoRead(canId, 0x606C, 0x00);
}

void EyouPhCan::requestStatusWord(uint16_t canId) const
{
    sendSdoRead(canId, 0x6041, 0x00);
}

bool EyouPhCan::setMode(MotorID motorId, MotorMode mode)
{
    if (!canController) {
        return false;
    }
    const uint16_t canId = static_cast<uint16_t>(motorId);
    registerManagedMotorId(canId);

    const uint8_t modeValue = (mode == MotorMode::Velocity) ? 0x03 : 0x01;
    sendSdoWriteU8(canId, 0x6060, 0x00, modeValue);

    std::lock_guard<std::mutex> lock(stateMutex);
    motorStates[canId].mode = mode;
    return true;
}

bool EyouPhCan::setVelocity(MotorID motorId, int32_t velocity)
{
    if (!canController) {
        return false;
    }
    const uint16_t canId = static_cast<uint16_t>(motorId);
    registerManagedMotorId(canId);
    sendSdoWriteI32(canId, 0x60FF, 0x00, velocity);
    return true;
}

bool EyouPhCan::setAcceleration(MotorID motorId, int32_t acceleration)
{
    if (!canController) {
        return false;
    }
    const uint16_t canId = static_cast<uint16_t>(motorId);
    registerManagedMotorId(canId);
    sendSdoWriteI32(canId, 0x6083, 0x00, acceleration);
    return true;
}

bool EyouPhCan::setDeceleration(MotorID motorId, int32_t deceleration)
{
    if (!canController) {
        return false;
    }
    const uint16_t canId = static_cast<uint16_t>(motorId);
    registerManagedMotorId(canId);
    sendSdoWriteI32(canId, 0x6084, 0x00, deceleration);
    return true;
}

bool EyouPhCan::setPosition(MotorID motorId, int32_t position)
{
    if (!canController) {
        return false;
    }
    const uint16_t canId = static_cast<uint16_t>(motorId);
    registerManagedMotorId(canId);
    sendSdoWriteI32(canId, 0x607A, 0x00, position);
    return true;
}

bool EyouPhCan::Enable(MotorID motorId)
{
    if (!canController) {
        return false;
    }
    const uint16_t canId = static_cast<uint16_t>(motorId);
    registerManagedMotorId(canId);

    // 按常见 CiA402 状态机：Shutdown(0x06) -> SwitchOn(0x07) -> EnableOperation(0x0F)
    sendSdoWriteU16(canId, 0x6040, 0x00, 0x0006);
    sendSdoWriteU16(canId, 0x6040, 0x00, 0x0007);
    sendSdoWriteU16(canId, 0x6040, 0x00, 0x000F);

    std::lock_guard<std::mutex> lock(stateMutex);
    motorStates[canId].enabled = true;
    return true;
}

bool EyouPhCan::Disable(MotorID motorId)
{
    if (!canController) {
        return false;
    }
    const uint16_t canId = static_cast<uint16_t>(motorId);
    registerManagedMotorId(canId);

    // 与现有上位机代码保持一致：0x02 作为停机/失能控制字。
    sendSdoWriteU16(canId, 0x6040, 0x00, 0x0002);

    std::lock_guard<std::mutex> lock(stateMutex);
    motorStates[canId].enabled = false;
    return true;
}

bool EyouPhCan::Stop(MotorID motorId)
{
    if (!canController) {
        return false;
    }
    const uint16_t canId = static_cast<uint16_t>(motorId);
    registerManagedMotorId(canId);
    sendSdoWriteU16(canId, 0x6040, 0x00, 0x0002);
    return true;
}

int64_t EyouPhCan::getPosition(MotorID motorId) const
{
    const uint16_t canId = static_cast<uint16_t>(motorId);
    registerManagedMotorId(canId);
    {
        std::lock_guard<std::mutex> lock(stateMutex);
        const auto it = motorStates.find(canId);
        if (it != motorStates.end() && it->second.positionReceived) {
            return it->second.position;
        }
    }
    requestPosition(canId);
    return 0;
}

int16_t EyouPhCan::getCurrent(MotorID motorId) const
{
    const uint16_t canId = static_cast<uint16_t>(motorId);
    registerManagedMotorId(canId);
    {
        std::lock_guard<std::mutex> lock(stateMutex);
        const auto it = motorStates.find(canId);
        if (it != motorStates.end() && it->second.currentReceived) {
            return it->second.current;
        }
    }
    requestCurrent(canId);
    return 0;
}

int16_t EyouPhCan::getVelocity(MotorID motorId) const
{
    const uint16_t canId = static_cast<uint16_t>(motorId);
    registerManagedMotorId(canId);
    {
        std::lock_guard<std::mutex> lock(stateMutex);
        const auto it = motorStates.find(canId);
        if (it != motorStates.end() && it->second.velocityReceived) {
            return it->second.velocity;
        }
    }
    requestVelocity(canId);
    return 0;
}

bool EyouPhCan::isEnabled(MotorID motorId) const
{
    const uint16_t canId = static_cast<uint16_t>(motorId);
    registerManagedMotorId(canId);
    std::lock_guard<std::mutex> lock(stateMutex);
    const auto it = motorStates.find(canId);
    return (it != motorStates.end()) ? it->second.enabled : false;
}

bool EyouPhCan::hasFault(MotorID motorId) const
{
    const uint16_t canId = static_cast<uint16_t>(motorId);
    registerManagedMotorId(canId);
    std::lock_guard<std::mutex> lock(stateMutex);
    const auto it = motorStates.find(canId);
    return (it != motorStates.end()) ? it->second.fault : false;
}

void EyouPhCan::initializeMotorRefresh(const std::vector<MotorID> &motorIds)
{
    {
        std::lock_guard<std::mutex> lock(refreshMutex);
        refreshMotorIds.clear();
        managedMotorIds.clear();
        refreshMotorIds.reserve(motorIds.size());
        for (MotorID id : motorIds) {
            const uint16_t canId = static_cast<uint16_t>(id);
            refreshMotorIds.push_back(canId);
            managedMotorIds.insert(canId);
        }
    }

    if (motorIds.empty()) {
        stopRefreshLoop();
        return;
    }

    if (refreshLoopActive.load()) {
        return;
    }

    bool expected = false;
    if (!refreshLoopActive.compare_exchange_strong(expected, true)) {
        return;
    }

    refreshThread = std::thread([this]() {
        while (refreshLoopActive.load()) {
            refreshMotorStates();
            std::size_t motorCount = 0;
            {
                std::lock_guard<std::mutex> lock(refreshMutex);
                motorCount = refreshMotorIds.size();
            }
            std::this_thread::sleep_for(this->computeRefreshSleep(motorCount));
        }
    });
}

void EyouPhCan::refreshMotorStates()
{
    std::vector<uint16_t> ids;
    {
        std::lock_guard<std::mutex> lock(refreshMutex);
        ids = refreshMotorIds;
    }

    for (const uint16_t id : ids) {
        requestPosition(id);
        requestCurrent(id);
        requestVelocity(id);
        requestStatusWord(id);
    }
}

void EyouPhCan::stopRefreshLoop()
{
    refreshLoopActive.store(false);
    if (refreshThread.joinable()) {
        refreshThread.join();
    }
}

void EyouPhCan::handleResponse(const CanTransport::Frame &frame)
{
    if (frame.isExtended || frame.isRemoteRequest || frame.dlc < 8) {
        return;
    }

    const uint16_t canId = static_cast<uint16_t>(frame.id & 0xFFFF);
    if (!isManagedMotorId(canId)) {
        return;
    }

    const uint8_t cmd = frame.data[0];
    if (cmd != 0x43 && cmd != 0x4B) {
        return;
    }

    const uint16_t index = static_cast<uint16_t>(frame.data[1]) |
                           (static_cast<uint16_t>(frame.data[2]) << 8);

    std::lock_guard<std::mutex> lock(stateMutex);
    MotorState &st = motorStates[canId];

    if (index == 0x6064 && cmd == 0x43) {
        st.position = readInt32LE(frame, 4);
        st.positionReceived = true;
        return;
    }
    if (index == 0x6077) {
        st.current = readInt16LE(frame, 4);
        st.currentReceived = true;
        return;
    }
    if (index == 0x606C && cmd == 0x43) {
        const int32_t raw = readInt32LE(frame, 4);
        st.velocity = static_cast<int16_t>(std::clamp<int32_t>(raw, -32768, 32767));
        st.velocityReceived = true;
        return;
    }
    if (index == 0x6041) {
        const uint16_t statusWord = static_cast<uint16_t>(frame.data[4]) |
                                    (static_cast<uint16_t>(frame.data[5]) << 8);
        st.enabled = (statusWord & 0x0004) != 0;
        st.fault = (statusWord & 0x0008) != 0;
        return;
    }
}
