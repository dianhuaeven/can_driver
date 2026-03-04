                                                                                                                                                                                                                                                                                                                                                                                                                                                #include <gtest/gtest.h>
#include <ros/time.h>

#include "can_driver/MtCan.h"
#include "can_driver/SocketCanController.h"

#include <memory>
#include <vector>

namespace {

class RosTimeFixture : public ::testing::Test {
protected:
    static void SetUpTestSuite()
    {
        ros::Time::init();
};

// Mock transport 用于捕获发送的帧
class MockTransport : public CanTransport {
public:
    void send(const Frame &frame) override
    {
        sentFrames.push_back(frame);
    }

    std::size_t addReceiveHandler(ReceiveHandler handler) override
    {
        receiveHandler = handler;
        return 1;
    }

    void removeReceiveHandler(std::size_t) override
    {
        receiveHandler = nullptr;
    }

    // 模拟接收帧
    void simulateReceive(const Frame &frame)
    {
        if (receiveHandler) {
            receiveHandler(frame);
        }
    }

    std::vector<Frame> sentFrames;
    ReceiveHandler receiveHandler;
};

} // namespace

// ============================================================================
// MT 协议编码测试
// ============================================================================

TEST_F(RosTimeFixture, MtCanEncodesPositionCommandCorrectly)
{
    auto mockTransport = std::make_shared<MockTransport>();
    MtCan mt(mockTransport);

    // 发送位置命令
    const MotorID motorId = static_cast<MotorID>(0x01);
    const int32_t position = 1000;

    mt.sendPositionCommand(motorId, position);

    // 验证发送了正确的帧
    ASSERT_EQ(mockTransport->sentFrames.size(), 1u);
    const auto &frame = mockTransport->sentFrames[0];

    // 验证 CAN ID（根据 MT 协议规范）
    // 假设 MT 协议的位置命令 ID 是 0x141 + motorId
    EXPECT_EQ(frame.id, 0x141u);
    EXPECT_TRUE(frame.isExtended);
    EXPECT_FALSE(frame.isRemoteRequest);

    // 验证数据长度
    EXPECT_EQ(frame.dlc, 8u);

    // 验证数据内容（根据 MT 协议规范）
    // 这里需要根据实际协议填充
    // 例如：前 4 字节是位置值（小端序）
    const int32_t decodedPos = static_cast<int32_t>(
        frame.data[0] | (frame.data[1] << 8) | (frame.data[2] << 16) | (frame.data[3] << 24));
    EXPECT_EQ(decodedPos, position);
}

TEST_F(RosTimeFixture, MtCanEncodesVelocityCommandCorrectly)
{
    auto mockTransport = std::make_shared<MockTransport>();
    MtCan mt(mockTransport);

    const MotorID motorId = static_cast<MotorID>(0x02);
    const int32_t velocity = 500;

    mt.sendVelocityCommand(motorId, velocity);

    ASSERT_EQ(mockTransport->sentFrames.size(), 1u);
    const auto &frame = mockTransport->sentFrames[0];

    // 验证速度命令的 CAN ID
    EXPECT_EQ(frame.id, 0x142u); // 假设速度命令 ID
    EXPECT_EQ(frame.dlc, 8u);
}

// ============================================================================
// MT 协议解码测试
// ============================================================================

TEST_F(RosTimeFixture, MtCanDecodesPositionFeedbackCorrectly)
{
    auto mockTransport = std::make_shared<MockTransport>();
    MtCan mt(mockTransport);

    // 注册电机
    std::vector<MotorID> motors = {static_cast<MotorID>(0x01)};
    mt.startRefresh(motors);

    // 模拟接收位置反馈帧
    CanTransport::Frame frame;
    frame.id = 0x241; // 假设位置反馈 ID
    frame.isExtended = true;
    frame.isRemoteRequest = false;
    frame.dlc = 8;

    // 编码位置值 2000（小端序）
    const int32_t expectedPos = 2000;
    frame.data[0] = expectedPos & 0xFF;
    frame.data[1] = (expectedPos >> 8) & 0xFF;
    frame.data[2] = (expectedPos >> 16) & 0xFF;
    frame.data[3] = (expectedPos >> 24) & 0xFF;

    // 模拟接收
    mockTransport->simulateReceive(frame);

    // 验证解码后的位置
    const int32_t decodedPos = mt.getPosition(static_cast<MotorID>(0x01));
    EXPECT_EQ(decodedPos, expectedPos);
}

TEST_F(RosTimeFixture, MtCanDecodesVelocityFeedbackCorrectly)
{
    auto mockTransport = std::make_shared<MockTransport>();
    MtCan mt(mockTransport);

    std::vector<MotorID> motors = {static_cast<MotorID>(0x01)};
    mt.startRefresh(motors);

    // 模拟接收速度反馈帧
    CanTransport::Frame frame;
    frame.id = 0x242; // 假设速度反馈 ID
    frame.isExtended = true;
    frame.dlc = 8;

    const int32_t expectedVel = -300;
    frame.data[0] = expectedVel & 0xFF;
    frame.data[1] = (expectedVel >> 8) & 0xFF;
    frame.data[2] = (expectedVel >> 16) & 0xFF;
    frame.data[3] = (expectedVel >> 24) & 0xFF;

    mockTransport->simulateReceive(frame);

    const int32_t decodedVel = mt.getVelocity(static_cast<MotorID>(0x01));
    EXPECT_EQ(decodedVel, expectedVel);
}

// ============================================================================
// 边界情况测试
// ============================================================================

TEST_F(RosTimeFixture, MtCanHandlesMaxInt32Position)
{
    auto mockTransport = std::make_shared<MockTransport>();
    MtCan mt(mockTransport);

    const int32_t maxPos = std::numeric_limits<int32_t>::max();
    mt.sendPositionCommand(static_cast<MotorID>(0x01), maxPos);

    ASSERT_EQ(mockTransport->sentFrames.size(), 1u);
    const auto &frame = mockTransport->sentFrames[0];

    const int32_t decoded = static_cast<int32_t>(
        frame.data[0] | (frame.data[1] << 8) | (frame.data[2] << 16) | (frame.data[3] << 24));
    EXPECT_EQ(decoded, maxPos);
}

TEST_F(RosTimeFixture, MtCanHandlesMinInt32Position)
{
    auto mockTransport = std::make_shared<MockTransport>();
    MtCan mt(mockTransport);

    const int32_t minPos = std::numeric_limits<int32_t>::min();
    mt.sendPositionCommand(static_cast<MotorID>(0x01), minPos);

    ASSERT_EQ(mockTransport->sentFrames.size(), 1u);
    const auto &frame = mockTransport->sentFrames[0];

    const int32_t decoded = static_cast<int32_t>(
        frame.data[0] | (frame.data[1] << 8) | (frame.data[2] << 16) | (frame.data[3] << 24));
    EXPECT_EQ(decoded, minPos);
}

TEST_F(RosTimeFixture, MtCanIgnoresUnknownMotorId)
{
    auto mockTransport = std::make_shared<MockTransport>();
    MtCan mt(mockTransport);

    // 只注册 motor 0x01
    std::vector<MotorID> motors = {static_cast<MotorID>(0x01)};
    mt.startRefresh(motors);

    // 接收来自未注册电机 0x02 的帧
    CanTransport::Frame frame;
    frame.id = 0x242; // motor 0x02 的反馈
    frame.dlc = 8;
    frame.data[0] = 100;

    mockTransport->simulateReceive(frame);

    // 验证未注册电机的数据不会影响已注册电机
    const int32_t pos = mt.getPosition(static_cast<MotorID>(0x01));
    EXPECT_EQ(pos, 0); // 应该保持初始值
}

// ============================================================================
// 多电机测试
// ============================================================================

TEST_F(RosTimeFixture, MtCanHandlesMultipleMotors)
{
    auto mockTransport = std::make_shared<MockTransport>();
    MtCan mt(mockTransport);

    std::vector<MotorID> motors = {
        static_cast<MotorID>(0x01),
        static_cast<MotorID>(0x02),
        static_cast<MotorID>(0x03)
    };
    mt.startRefresh(motors);

    // 发送不同电机的位置反馈
    for (size_t i = 0; i < motors.size(); ++i) {
        CanTransport::Frame frame;
        frame.id = 0x241 + i; // 不同电机的 ID
        frame.dlc = 8;

        const int32_t pos = 1000 * (i + 1);
        frame.data[0] = pos & 0xFF;
        frame.data[1] = (pos >> 8) & 0xFF;
        frame.data[2] = (pos >> 16) & 0xFF;
        frame.data[3] = (pos >> 24) & 0xFF;

        mockTransport->simulateReceive(frame);
    }

    // 验证每个电机的位置都正确
    EXPECT_EQ(mt.getPosition(static_cast<MotorID>(0x01)), 1000);
    EXPECT_EQ(mt.getPosition(static_cast<MotorID>(0x02)), 2000);
    EXPECT_EQ(mt.getPosition(static_cast<MotorID>(0x03)), 3000);
}

}