#include <gtest/gtest.h>
#include <ros/time.h>

#include "can_driver/EyouPhCan.h"

#include <cstdint>
#include <memory>
#include <vector>

namespace {

class MockTransport : public CanTransport {
public:
    void send(const Frame &frame) override
    {
        sentFrames.push_back(frame);
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

    std::vector<Frame> sentFrames;
    ReceiveHandler receiveHandler;
};

class EyouPhCanTest : public ::testing::Test {
protected:
    static void SetUpTestSuite()
    {
        ros::Time::init();
    }

    EyouPhCanTest()
        : transport(std::make_shared<MockTransport>())
        , proto(transport)
    {
    }

    std::shared_ptr<MockTransport> transport;
    EyouPhCan proto;
};

} // namespace

TEST_F(EyouPhCanTest, SetVelocityUsesSdoWrite6040StyleFrame)
{
    constexpr MotorID kMotorId = static_cast<MotorID>(0x601);
    constexpr int32_t kVel = 700000;

    ASSERT_TRUE(proto.setVelocity(kMotorId, kVel));
    ASSERT_EQ(transport->sentFrames.size(), 1u);

    const auto &f = transport->sentFrames[0];
    EXPECT_EQ(f.id, 0x601u);
    EXPECT_EQ(f.dlc, 8u);
    EXPECT_EQ(f.data[0], 0x23);
    EXPECT_EQ(f.data[1], 0xFF);
    EXPECT_EQ(f.data[2], 0x60);
    EXPECT_EQ(f.data[3], 0x00);
    EXPECT_EQ(f.data[4], static_cast<uint8_t>(kVel & 0xFF));
    EXPECT_EQ(f.data[5], static_cast<uint8_t>((kVel >> 8) & 0xFF));
    EXPECT_EQ(f.data[6], static_cast<uint8_t>((kVel >> 16) & 0xFF));
    EXPECT_EQ(f.data[7], static_cast<uint8_t>((kVel >> 24) & 0xFF));
}

TEST_F(EyouPhCanTest, HandleReadPositionResponseUpdatesCache)
{
    CanTransport::Frame frame{};
    frame.id = 0x601;
    frame.isExtended = false;
    frame.isRemoteRequest = false;
    frame.dlc = 8;
    frame.data[0] = 0x43;
    frame.data[1] = 0x64;
    frame.data[2] = 0x60;
    frame.data[3] = 0x00;
    frame.data[4] = 0x78;
    frame.data[5] = 0x56;
    frame.data[6] = 0x34;
    frame.data[7] = 0x12;

    transport->simulateReceive(frame);

    EXPECT_EQ(proto.getPosition(static_cast<MotorID>(0x601)), 0x12345678);
}

TEST_F(EyouPhCanTest, EnableWritesControlWordSequence)
{
    constexpr MotorID kMotorId = static_cast<MotorID>(0x601);
    ASSERT_TRUE(proto.Enable(kMotorId));
    ASSERT_EQ(transport->sentFrames.size(), 3u);

    EXPECT_EQ(transport->sentFrames[0].data[0], 0x2B);
    EXPECT_EQ(transport->sentFrames[0].data[1], 0x40);
    EXPECT_EQ(transport->sentFrames[0].data[2], 0x60);
    EXPECT_EQ(transport->sentFrames[0].data[4], 0x06);

    EXPECT_EQ(transport->sentFrames[1].data[4], 0x07);
    EXPECT_EQ(transport->sentFrames[2].data[4], 0x0F);
}
