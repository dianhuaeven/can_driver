#include "can_driver/SocketCanController.h"
#include <fcntl.h>
#include <ros/console.h>
#include <errno.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>
#include <algorithm>
#include <vector>

namespace {

bool setNonBlocking(int fd) {
  const int flags = ::fcntl(fd, F_GETFL, 0);
  if (flags < 0) {
    return false;
  }
  return ::fcntl(fd, F_SETFL, flags | O_NONBLOCK) == 0;
}

} // namespace

SocketCanController::SocketCanController() {
}

SocketCanController::~SocketCanController() {
  shutdown();
}

bool SocketCanController::initialize(const std::string &device, bool loopback) {
  // 允许重复初始化：先清理旧 socket 和线程。
  shutdown();

  const int fd = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd < 0) {
    ROS_ERROR_STREAM("[SocketCanController] Failed to create CAN socket for " << device << ": errno=" << errno);
    return false;
  }

  if (!setNonBlocking(fd)) {
    ROS_ERROR_STREAM("[SocketCanController] Failed to set non-blocking mode on "
                     << device << ": errno=" << errno);
    ::close(fd);
    return false;
  }

  const int loopbackOpt = shouldEnableLocalLoopback() ? 1 : 0;
  if (::setsockopt(fd, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopbackOpt, sizeof(loopbackOpt)) < 0) {
    ROS_WARN_STREAM("[SocketCanController] Failed to set loopback option on " << device << ", errno=" << errno << ". Continue with system default.");
  }

  const int recvOwnMsgs = shouldReceiveOwnMessages(loopback) ? 1 : 0;
  if (::setsockopt(fd, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &recvOwnMsgs, sizeof(recvOwnMsgs)) < 0) {
    ROS_WARN_STREAM("[SocketCanController] Failed to set RECV_OWN_MSGS on " << device << ", errno=" << errno);
  }

  struct sockaddr_can addr {};
  addr.can_family = AF_CAN;
  addr.can_ifindex = static_cast<int>(::if_nametoindex(device.c_str()));
  if (addr.can_ifindex == 0) {
    ROS_ERROR_STREAM("[SocketCanController] Unknown CAN device " << device);
    ::close(fd);
    return false;
  }

  if (::bind(fd, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    ROS_ERROR_STREAM("[SocketCanController] Failed to bind " << device << ": errno=" << errno);
    ::close(fd);
    return false;
  }

  socketFd_ = fd;
  stopRequested_.store(false);
  receiveThread_ = std::thread(&SocketCanController::receiveLoop, this);
  deviceName_ = device;
  initialized_.store(true);
  return true;
}

void SocketCanController::shutdown() {
  stopRequested_.store(true);
  initialized_.store(false);
  if (receiveThread_.joinable()) {
    receiveThread_.join();
  }
  if (socketFd_ >= 0) {
    ::close(socketFd_);
    socketFd_ = -1;
  }
  // handler ID 从 1 重新开始，便于测试中验证生命周期。
  {
    std::lock_guard<std::mutex> lock(handlerMutex_);
    handlers_.clear();
    nextHandlerId_.store(1);
  }
  deviceName_.clear();
}

bool SocketCanController::isReady() const {
  return initialized_.load();
}

std::string SocketCanController::device() const {
  return deviceName_;
}

void SocketCanController::send(const CanTransport::Frame &frame) {
  if (!initialized_.load() || socketFd_ < 0) {
    return;
  }

  const struct can_frame socketFrame = toLinuxCanFrame(frame);
  const auto written = ::send(socketFd_, &socketFrame, sizeof(socketFrame), MSG_DONTWAIT);

  if (written < 0) {
    const int errorCode = errno;
    if (isBackpressureSendError(errorCode)) {
      ROS_WARN_STREAM_THROTTLE(1.0,
                               "[SocketCanController] TX queue saturated on " << deviceName_
                               << ", dropping frame without blocking"
                               << " (errno=" << errorCode << ": "
                               << std::strerror(errorCode) << ")");
      return;
    }
    if (isLinkUnavailableSendError(errorCode)) {
      ROS_WARN_STREAM_THROTTLE(1.0,
                               "[SocketCanController] CAN link unavailable on " << deviceName_
                               << ", dropping frame"
                               << " (errno=" << errorCode << ": "
                               << std::strerror(errorCode) << ")");
      return;
    }

    ROS_ERROR_STREAM_THROTTLE(1.0,
                              "[SocketCanController] send() failed on " << deviceName_
                              << " (errno=" << errorCode << ": "
                              << std::strerror(errorCode) << ")");
    return;
  }

  if (written != static_cast<ssize_t>(sizeof(socketFrame))) {
    ROS_ERROR_STREAM_THROTTLE(1.0,
                              "[SocketCanController] Partial send on " << deviceName_
                              << ": " << written << " bytes");
  }
}

bool SocketCanController::shouldEnableLocalLoopback() {
  // 保持内核本地 loopback 打开，这样同机的 candump/诊断工具能看到主站发出的帧。
  return true;
}

bool SocketCanController::shouldReceiveOwnMessages(bool loopback) {
  // 只有显式请求 loopback 测试时，当前 socket 才接收自己发出的帧。
  return loopback;
}

bool SocketCanController::isBackpressureSendError(int errorCode) {
  return errorCode == EAGAIN || errorCode == EWOULDBLOCK || errorCode == ENOBUFS;
}

bool SocketCanController::isLinkUnavailableSendError(int errorCode) {
  return errorCode == ENETDOWN || errorCode == ENODEV || errorCode == ENXIO;
}

void SocketCanController::receiveLoop() {
  while (!stopRequested_.load()) {
    if (socketFd_ < 0) {
      break;
    }

    fd_set readFds;
    FD_ZERO(&readFds);
    FD_SET(socketFd_, &readFds);

    struct timeval tv {};
    tv.tv_sec = 0;
    tv.tv_usec = 200000; // 200ms，保证 shutdown 最多等待一个轮询周期

    const int sel = ::select(socketFd_ + 1, &readFds, nullptr, nullptr, &tv);
    if (sel < 0) {
      if (errno == EINTR) {
        continue;
      }
      ROS_WARN_STREAM("[SocketCanController] select() failed on " << deviceName_ << ", errno=" << errno);
      continue;
    }

    if (sel == 0 || !FD_ISSET(socketFd_, &readFds)) {
      continue;
    }

    struct can_frame rxFrame {};
    const auto n = ::read(socketFd_, &rxFrame, sizeof(rxFrame));
    if (n == static_cast<ssize_t>(sizeof(rxFrame))) {
      dispatchReceive(fromLinuxCanFrame(rxFrame));
      continue;
    }

    if (n < 0) {
      const int errorCode = errno;
      if (errorCode == EAGAIN || errorCode == EWOULDBLOCK || errorCode == EINTR) {
        continue;
      }
      ROS_WARN_STREAM_THROTTLE(1.0,
                               "[SocketCanController] read() failed on " << deviceName_
                               << " (errno=" << errorCode << ": "
                               << std::strerror(errorCode) << ")");
      continue;
    }

    ROS_WARN_STREAM_THROTTLE(1.0,
                             "[SocketCanController] Short read on " << deviceName_
                             << ": " << n << " bytes");
  }
}

std::size_t SocketCanController::addReceiveHandler(ReceiveHandler handler) {
  if (!handler) {
    return 0;
  }
  std::lock_guard<std::mutex> lock(handlerMutex_);
  const std::size_t id = nextHandlerId_++;
  handlers_.emplace(id, std::move(handler));
  return id;
}

void SocketCanController::removeReceiveHandler(std::size_t handlerId) {
  if (handlerId == 0) {
    return;
  }
  std::lock_guard<std::mutex> lock(handlerMutex_);
  handlers_.erase(handlerId);
}

void SocketCanController::dispatchReceive(const CanTransport::Frame &frame) {
  std::vector<ReceiveHandler> handlersCopy;
  {
    std::lock_guard<std::mutex> lock(handlerMutex_);
    handlersCopy.reserve(handlers_.size());
    for (const auto &entry : handlers_) {
      handlersCopy.push_back(entry.second);
    }
  }
  // 在锁外回调，避免 handler 内部再次注册/注销造成死锁。
  for (auto &handler : handlersCopy) {
    if (handler) {
      handler(frame);
    }
  }
}

struct can_frame SocketCanController::toLinuxCanFrame(const CanTransport::Frame &frame) const {
  struct can_frame socketFrame {};
  socketFrame.can_id = frame.id;
  if (frame.isExtended) {
    socketFrame.can_id |= CAN_EFF_FLAG;
  }
  if (frame.isRemoteRequest) {
    socketFrame.can_id |= CAN_RTR_FLAG;
  }
  socketFrame.can_dlc = std::min<std::uint8_t>(frame.dlc, static_cast<std::uint8_t>(8));
  for (std::size_t i = 0; i < socketFrame.can_dlc; ++i) {
    socketFrame.data[i] = frame.data[i];
  }
  return socketFrame;
}

CanTransport::Frame SocketCanController::fromLinuxCanFrame(const struct can_frame &frame) const {
  CanTransport::Frame userFrame;
  userFrame.id = static_cast<std::uint32_t>(frame.can_id & CAN_EFF_MASK);
  userFrame.isExtended = (frame.can_id & CAN_EFF_FLAG) != 0;
  userFrame.isRemoteRequest = (frame.can_id & CAN_RTR_FLAG) != 0;
  userFrame.data.fill(0);
  userFrame.dlc = std::min<std::uint8_t>(frame.can_dlc, static_cast<std::uint8_t>(8));
  for (std::size_t i = 0; i < userFrame.dlc; ++i) {
    userFrame.data[i] = frame.data[i];
  }
  return userFrame;
}
