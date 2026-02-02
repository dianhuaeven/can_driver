#include "SocketCanController.h"

#include <ros/console.h>
#include <socketcan_interface/threading.h>
#include <socketcan_interface/settings.h>

#include <algorithm>
#include <cstring>
#include <functional>

SocketCanController::SocketCanController()
    : interface_(std::make_shared<can::ThreadedSocketCANInterface>())
{
}

SocketCanController::~SocketCanController()
{
    shutdown();
}

bool SocketCanController::initialize(const std::string &device, bool loopback)
{
    shutdown();

    if (!interface_) {
        interface_ = std::make_shared<can::ThreadedSocketCANInterface>();
    }

    if (!interface_->init(device, loopback, can::NoSettings::create())) {
        ROS_ERROR_STREAM("[SocketCanController] Failed to init device " << device);
        return false;
    }

    frameListener_ = interface_->createMsgListener(
        std::bind(&SocketCanController::handleFrame, this, std::placeholders::_1));

    stateListener_ = interface_->createStateListener(
        [device](const can::State &state) {
            if (!state.isReady()) {
                ROS_WARN_STREAM("[SocketCanController] Device " << device
                                << " entered state " << state.driver_state
                                << " error=" << state.error_code.message()
                                << " internal=" << state.internal_error);
            }
        });

    deviceName_ = device;
    initialized_.store(true);
    return true;
}

void SocketCanController::shutdown()
{
    initialized_.store(false);
    frameListener_.reset();
    stateListener_.reset();

    if (interface_) {
        interface_->shutdown();
    }

    {
        std::lock_guard<std::mutex> lock(handlerMutex_);
        handlers_.clear();
        nextHandlerId_.store(1);
    }
    deviceName_.clear();
}

bool SocketCanController::isReady() const
{
    return initialized_.load();
}

std::string SocketCanController::device() const
{
    return deviceName_;
}

void SocketCanController::send(const CanTransport::Frame &frame)
{
    if (!initialized_.load() || !interface_) {
        return;
    }

    const can::Frame socketFrame = toSocketCanFrame(frame);
    if (!socketFrame.isValid()) {
        ROS_WARN_STREAM("[SocketCanController] Skip invalid frame for device " << deviceName_);
        return;
    }

    if (!interface_->send(socketFrame)) {
        ROS_WARN_STREAM("[SocketCanController] Failed to enqueue frame on " << deviceName_);
    }
}

std::size_t SocketCanController::addReceiveHandler(ReceiveHandler handler)
{
    if (!handler) {
        return 0;
    }

    std::lock_guard<std::mutex> lock(handlerMutex_);
    const std::size_t id = nextHandlerId_++;
    handlers_.emplace(id, std::move(handler));
    return id;
}

void SocketCanController::removeReceiveHandler(std::size_t handlerId)
{
    if (handlerId == 0) {
        return;
    }

    std::lock_guard<std::mutex> lock(handlerMutex_);
    handlers_.erase(handlerId);
}

void SocketCanController::handleFrame(const can::Frame &frame)
{
    dispatchReceive(fromSocketCanFrame(frame));
}

void SocketCanController::dispatchReceive(const CanTransport::Frame &frame)
{
    std::unordered_map<std::size_t, ReceiveHandler> handlersCopy;
    {
        std::lock_guard<std::mutex> lock(handlerMutex_);
        handlersCopy = handlers_;
    }

    for (auto &entry : handlersCopy) {
        if (entry.second) {
            entry.second(frame);
        }
    }
}

can::Frame SocketCanController::toSocketCanFrame(const CanTransport::Frame &frame) const
{
    can::Frame socketFrame;
    socketFrame.id = frame.id;
    socketFrame.is_extended = frame.isExtended ? 1 : 0;
    socketFrame.is_rtr = frame.isRemoteRequest ? 1 : 0;
    socketFrame.is_error = 0;
    socketFrame.dlc = std::min<std::uint8_t>(frame.dlc, static_cast<std::uint8_t>(socketFrame.data.size()));
    for (std::size_t i = 0; i < socketFrame.dlc; ++i) {
        socketFrame.data[i] = frame.data[i];
    }
    return socketFrame;
}

CanTransport::Frame SocketCanController::fromSocketCanFrame(const can::Frame &frame) const
{
    CanTransport::Frame userFrame;
    userFrame.id = frame.id;
    userFrame.isExtended = frame.is_extended != 0;
    userFrame.isRemoteRequest = frame.is_rtr != 0;
    userFrame.dlc = std::min<std::uint8_t>(frame.dlc, static_cast<std::uint8_t>(userFrame.data.size()));
    for (std::size_t i = 0; i < userFrame.dlc; ++i) {
        userFrame.data[i] = frame.data[i];
    }
    return userFrame;
}
