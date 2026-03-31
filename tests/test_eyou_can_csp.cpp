#include <gtest/gtest.h>
#include <ros/time.h>

#include "can_driver/EyouCan.h"

#include <cstdint>
#include <memory>
#include <vector>

namespace {

// 轻量 mock：只验证协议层编码/解码，不依赖真实 socketcan。
class MockTransport : public CanTransport {
public:
    SendResult send(const Frame &frame) override
    {
        sentFrames.push_back(frame);
        return SendResult::Ok;
    }

    std::size_t addReceiveHandler(ReceiveHandler handler) override
    {
        receiveHandler = std::move(handler);
        return 1;
    }

    void removeReceiveHandler(std::size_t) override
    {
        receiveHandler = nullptr;
    }

    void simulateReceive(const Frame &frame) const
    {
        if (receiveHandler) {
            receiveHandler(frame);
        }
    }

    void clearSent()
    {
        sentFrames.clear();
    }

    std::vector<Frame> sentFrames;
    ReceiveHandler receiveHandler;
};

class MockTxDispatcher : public CanTxDispatcher {
public:
    explicit MockTxDispatcher(std::shared_ptr<MockTransport> transport)
        : transport(std::move(transport))
    {
    }

    void submit(const Request &request) override
    {
        requests.push_back(request);
        if (transport) {
            transport->send(request.frame);
        }
    }

    std::shared_ptr<MockTransport> transport;
    std::vector<Request> requests;
};

class EyouCanCSPTest : public ::testing::Test {
protected:
    static void SetUpTestSuite()
    {
        ros::Time::init();
    }

    EyouCanCSPTest()
        : transport(std::make_shared<MockTransport>())
        , txDispatcher(std::make_shared<MockTxDispatcher>(transport))
        , eyou(transport, txDispatcher)
    {
    }

    std::shared_ptr<MockTransport> transport;
    std::shared_ptr<MockTxDispatcher> txDispatcher;
    EyouCan eyou;
};

} // namespace

TEST_F(EyouCanCSPTest, QuickSetPositionGeneratesCorrectFrame)
{
    constexpr MotorID kMotorId = static_cast<MotorID>(0x05);
    constexpr int32_t kPosition = 0x01020304;

    ASSERT_TRUE(eyou.quickSetPosition(kMotorId, kPosition));
    ASSERT_EQ(transport->sentFrames.size(), 1u);
    ASSERT_EQ(txDispatcher->requests.size(), 1u);

    const auto &frame = transport->sentFrames[0];
    // CSP 模式：CMD=0x05 快写命令 + 0x0A 位置寄存器
    EXPECT_EQ(frame.id, 0x0005u);
    EXPECT_FALSE(frame.isExtended);
    EXPECT_FALSE(frame.isRemoteRequest);
    EXPECT_EQ(frame.dlc, 8u);
    EXPECT_EQ(frame.data[0], 0x05); // quick write command
    EXPECT_EQ(frame.data[1], 0x0A); // position register
    EXPECT_EQ(frame.data[2], 0x01); // position value (big-endian)
    EXPECT_EQ(frame.data[3], 0x02);
    EXPECT_EQ(frame.data[4], 0x03);
    EXPECT_EQ(frame.data[5], 0x04);
    EXPECT_EQ(frame.data[6], 0x00);
    EXPECT_EQ(frame.data[7], 0x00);
}

TEST_F(EyouCanCSPTest, CSPModeCanBeSet)
{
    constexpr MotorID kMotorId = static_cast<MotorID>(0x05);

    ASSERT_TRUE(eyou.setMode(kMotorId, CanProtocol::MotorMode::CSP));
    ASSERT_EQ(transport->sentFrames.size(), 1u);
    ASSERT_EQ(txDispatcher->requests.size(), 1u);

    const auto &frame = transport->sentFrames[0];
    // 模式设置：CMD=0x01 + 0x0F 模式寄存器 + 模式值 5
    EXPECT_EQ(frame.id, 0x0005u);
    EXPECT_EQ(frame.dlc, 8u);
    EXPECT_EQ(frame.data[0], 0x01); // write command
    EXPECT_EQ(frame.data[1], 0x0F); // mode register
    EXPECT_EQ(frame.data[2], 0x00); // mode value = 5 (big-endian)
    EXPECT_EQ(frame.data[3], 0x00);
    EXPECT_EQ(frame.data[4], 0x00);
    EXPECT_EQ(frame.data[5], 0x05); // CSP mode
}

TEST_F(EyouCanCSPTest, QuickSetPositionOnlyOneFrame)
{
    constexpr MotorID kMotorId = static_cast<MotorID>(0x05);

    // CSP 模式应该只发送一帧（快写位置），不像 PP 模式需要多帧
    ASSERT_TRUE(eyou.quickSetPosition(kMotorId, 1000));
    EXPECT_EQ(transport->sentFrames.size(), 1u);

    transport->clearSent();
    txDispatcher->requests.clear();
    ASSERT_TRUE(eyou.quickSetPosition(kMotorId, 2000));
    EXPECT_EQ(transport->sentFrames.size(), 1u);
    EXPECT_EQ(txDispatcher->requests.size(), 1u);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
