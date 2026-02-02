#include <gtest/gtest.h>
#include <can_driver/SocketCanController.h>

// Friend accessor to reach private methods for unit testing
class SocketCanControllerTestAccessor {
public:
  static can::Frame toSock(SocketCanController& c, const CanTransport::Frame& f){ return c.toSocketCanFrame(f); }
  static CanTransport::Frame fromSock(SocketCanController& c, const can::Frame& f){ return c.fromSocketCanFrame(f); }
  static void dispatch(SocketCanController& c, const CanTransport::Frame& f){ c.dispatchReceive(f); }
};

TEST(SocketCanController, EncodeDecodeRoundtripAndBounds){
  SocketCanController ctrl;
  CanTransport::Frame f{}; f.id=0x123; f.isExtended=true; f.isRemoteRequest=false; f.dlc=10; // >8
  for(size_t i=0;i<f.data.size();++i) f.data[i]=static_cast<uint8_t>(i+1);

  auto sf = SocketCanControllerTestAccessor::toSock(ctrl,f);
  EXPECT_EQ(sf.id,f.id);
  EXPECT_EQ(sf.is_extended,1);
  EXPECT_EQ(sf.is_rtr,0);
  EXPECT_LE(sf.dlc,8);
  for(size_t i=0;i<sf.dlc;++i) EXPECT_EQ(sf.data[i], f.data[i]);

  auto back = SocketCanControllerTestAccessor::fromSock(ctrl,sf);
  EXPECT_EQ(back.id,f.id);
  EXPECT_TRUE(back.isExtended);
  EXPECT_FALSE(back.isRemoteRequest);
  EXPECT_EQ(back.dlc,sf.dlc);
  for(size_t i=0;i<back.dlc;++i) EXPECT_EQ(back.data[i], sf.data[i]);
}

TEST(SocketCanController, HandlerLifecycleAndDispatch){
  SocketCanController ctrl;
  size_t called=0;
  auto id = ctrl.addReceiveHandler([&](const CanTransport::Frame&){ ++called; });
  EXPECT_NE(id,0u);

  CanTransport::Frame f{}; f.id=1; f.dlc=1; f.data[0]=0xAA;
  SocketCanControllerTestAccessor::dispatch(ctrl,f);
  EXPECT_EQ(called,1u);

  ctrl.removeReceiveHandler(id);
  SocketCanControllerTestAccessor::dispatch(ctrl,f);
  EXPECT_EQ(called,1u);

  ctrl.removeReceiveHandler(0); // no-op
}

TEST(SocketCanController, ShutdownResetsState){
  SocketCanController ctrl;
  auto id1 = ctrl.addReceiveHandler([](auto const&){});
  ctrl.removeReceiveHandler(id1);
  ctrl.shutdown();
  auto id2 = ctrl.addReceiveHandler([](auto const&){});
  EXPECT_EQ(id2,1u);
}
