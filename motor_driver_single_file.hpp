#ifndef MOTOR_DRIVER_SINGLE_FILE_HPP
#define MOTOR_DRIVER_SINGLE_FILE_HPP

#include <QCoreApplication>
#include <QEventLoop>
#include <QHostAddress>
#include <QTime>
#include <QTimer>
#include <QUdpSocket>

#include <array>
#include <cstdint>

namespace rebot_motor {

// IDs (保持与原工程一致)
constexpr uint16_t swingArm1 = 0x601;
constexpr uint16_t swingArm2 = 0x602;
constexpr uint16_t swingArm3 = 0x603;
constexpr uint16_t swingArm4 = 0x604;

constexpr uint16_t MT_Wheel_1 = 0x14B;
constexpr uint16_t MT_Wheel_2 = 0x141;

constexpr uint16_t Arm_Eyou_1 = 0x50;
constexpr uint16_t Arm_Eyou_2 = 0x605;
constexpr uint16_t Arm_Eyou_3 = 0x51;
constexpr uint16_t Arm_Eyou_4 = 0x01;
constexpr uint16_t Arm_Eyou_5 = 0x53;
constexpr uint16_t Arm_Eyou_6 = 0x54;
constexpr uint16_t Arm_Eyou_7 = 0x02;

// 速度与方向 (保持与原工程一致)
constexpr int32_t Arm_Eyou_1_Vel = 5000;
constexpr int32_t Arm_Eyou_2_Vel = 300000;
constexpr int32_t Arm_Eyou_3_Vel = 5000;
constexpr int32_t Arm_Eyou_4_Vel = 5000;
constexpr int32_t Arm_Eyou_5_Vel = 5000;
constexpr int32_t Arm_Eyou_6_Vel = 5000;
constexpr int32_t Arm_Eyou_7_Vel = 3000;

constexpr int32_t Arm_Eyou_1_DIR = 1;
constexpr int32_t Arm_Eyou_2_DIR = 1;
constexpr int32_t Arm_Eyou_3_DIR = 1;
constexpr int32_t Arm_Eyou_4_DIR = 1;
constexpr int32_t Arm_Eyou_5_DIR = 1;
constexpr int32_t Arm_Eyou_6_DIR = 1;
constexpr int32_t Arm_Eyou_7_DIR = 1;

constexpr int32_t MT_Wheel_1_Vel_LOW = 20000;
constexpr int32_t MT_Wheel_2_Vel_LOW = 20000;
constexpr int32_t MT_Wheel_1_Vel_HIGH = 35000;
constexpr int32_t MT_Wheel_2_Vel_HIGH = 35000;
constexpr int32_t MT_Wheel_1_Vel_LEFT = 20000;
constexpr int32_t MT_Wheel_2_Vel_LEFT = 20000;
constexpr int32_t MT_Wheel_1_Vel_RIGHT = 20000;
constexpr int32_t MT_Wheel_2_Vel_RIGHT = 20000;

constexpr int32_t MT_Wheel_1_DIR = 1;
constexpr int32_t MT_Wheel_2_DIR = 1;

constexpr int32_t swingArm1_DIR = 1;
constexpr int32_t swingArm2_DIR = 1;
constexpr int32_t swingArm3_DIR = 1;
constexpr int32_t swingArm4_DIR = 1;

constexpr int32_t swingArm1_Vel = 700000;
constexpr int32_t swingArm2_Vel = 700000;
constexpr int32_t swingArm3_Vel = 700000;
constexpr int32_t swingArm4_Vel = 700000;

inline void spinSleep(unsigned int msec) {
    const QTime dieTime = QTime::currentTime().addMSecs(static_cast<int>(msec));
    while (QTime::currentTime() < dieTime) {
        QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
    }
}

class EyouPHDriver final : public QObject {
public:
    explicit EyouPHDriver(QObject* parent = nullptr) : QObject(parent) {
        udp_ = new QUdpSocket(this);
        udp_->bind(7101);
        serverIp_.setAddress("192.168.1.253");
        serverPort_ = 1031;

        readTimer_ = new QTimer(this);
        readTimer_->setInterval(50);

        QObject::connect(readTimer_, &QTimer::timeout, this, &EyouPHDriver::readTimerFun);
        QObject::connect(udp_, &QUdpSocket::readyRead, this, &EyouPHDriver::receiveData);
    }

    void openReadTimer() { readTimer_->start(); }

    void setStop02(uint16_t id) { writeControlWord(id, 0x02); }
    void setEnable1F(uint16_t id) { writeControlWord(id, 0x1F); }
    void setShutdown06(uint16_t id) { writeControlWord(id, 0x06); }
    void setSwitchOn07(uint16_t id) { writeControlWord(id, 0x07); }
    void setSwitchOnEnable0F(uint16_t id) { writeControlWord(id, 0x0F); }

    void setVelocityMode(uint16_t id) {
        QByteArray a(13, 0x00);
        a[0] = 0x08;
        a[3] = static_cast<char>((id >> 8) & 0xFF);
        a[4] = static_cast<char>(id & 0xFF);
        a[5] = 0x2F;
        a[6] = 0x60;
        a[7] = 0x60;
        a[9] = 0x03;
        udp_->writeDatagram(a, serverIp_, serverPort_);
    }

    void setPositionMode(uint16_t id) {
        QByteArray a(13, 0x00);
        a[0] = 0x08;
        a[3] = static_cast<char>((id >> 8) & 0xFF);
        a[4] = static_cast<char>(id & 0xFF);
        a[5] = 0x2F;
        a[6] = 0x60;
        a[7] = 0x60;
        a[9] = 0x01;
        udp_->writeDatagram(a, serverIp_, serverPort_);
    }

    void setPosition(uint16_t id, int32_t data) {
        QByteArray a(13, 0x00);
        a[0] = 0x08;
        a[3] = static_cast<char>((id >> 8) & 0xFF);
        a[4] = static_cast<char>(id & 0xFF);
        a[5] = 0x23;
        a[6] = 0x7A;
        a[7] = 0x60;
        a[8] = static_cast<char>((data >> 24) & 0xFF);
        a[9] = static_cast<char>((data >> 16) & 0xFF);
        a[10] = static_cast<char>((data >> 8) & 0xFF);
        a[11] = static_cast<char>(data & 0xFF);
        udp_->writeDatagram(a, serverIp_, serverPort_);
    }

    // 按原项目写法，速度写入顺序保持不变（低字节在前）。
    void setVelocity(uint16_t id, int32_t data) {
        QByteArray a(13, 0x00);
        a[0] = 0x08;
        a[3] = static_cast<char>((id >> 8) & 0xFF);
        a[4] = static_cast<char>(id & 0xFF);
        a[5] = 0x23;
        a[6] = static_cast<char>(0xFF);
        a[7] = 0x60;
        a[8] = 0x00;
        a[9] = static_cast<char>(data & 0xFF);
        a[10] = static_cast<char>((data >> 8) & 0xFF);
        a[11] = static_cast<char>((data >> 16) & 0xFF);
        a[12] = static_cast<char>((data >> 24) & 0xFF);
        udp_->writeDatagram(a, serverIp_, serverPort_);
    }

    void setPVelocity(uint16_t id, uint32_t data) {
        QByteArray a(13, 0x00);
        a[0] = 0x08;
        a[3] = static_cast<char>((id >> 8) & 0xFF);
        a[4] = static_cast<char>(id & 0xFF);
        a[5] = 0x23;
        a[6] = static_cast<char>(0x81);
        a[7] = 0x60;
        a[8] = 0x00;
        a[9] = static_cast<char>(data & 0xFF);
        a[10] = static_cast<char>((data >> 8) & 0xFF);
        a[11] = static_cast<char>((data >> 16) & 0xFF);
        a[12] = static_cast<char>((data >> 24) & 0xFF);
        udp_->writeDatagram(a, serverIp_, serverPort_);
    }

    void setAcceleration(uint16_t id, uint32_t data) {
        writeU32(id, 0x23, 0x83, 0x60, data);
    }

    void setDeceleration(uint16_t id, uint32_t data) {
        writeU32(id, 0x23, 0x84, 0x60, data);
    }

    const std::array<int32_t, 6>& positions() const { return position_; }
    const std::array<int16_t, 6>& currents() const { return current_; }

    void readTimerFun() {
        switch (readTimerNum_) {
        case 1:
            readPosition(swingArm1);
            spinSleep(50);
            readCurrent(swingArm1);
            break;
        case 2:
            readPosition(swingArm2);
            spinSleep(50);
            readCurrent(swingArm2);
            break;
        case 3:
            readPosition(swingArm3);
            spinSleep(50);
            readCurrent(swingArm3);
            break;
        case 4:
            readPosition(swingArm4);
            spinSleep(50);
            readCurrent(swingArm4);
            break;
        case 5:
            readPosition(Arm_Eyou_2);
            spinSleep(50);
            readCurrent(Arm_Eyou_2);
            readTimerNum_ = 0;
            break;
        default:
            break;
        }
        ++readTimerNum_;
    }

    void receiveData() {
        QByteArray a;
        a.resize(static_cast<int>(udp_->bytesAvailable()));
        udp_->readDatagram(a.data(), a.size());
        if (a.size() < 13) {
            return;
        }
        if (a[5] == static_cast<char>(0x4B)) {
            disposeReadCurrent(a);
        } else if (a[5] == static_cast<char>(0x43)) {
            disposeReadPosition(a);
        }
    }

private:
    void writeControlWord(uint16_t id, uint8_t value) {
        QByteArray a(13, 0x00);
        a[0] = 0x08;
        a[3] = static_cast<char>((id >> 8) & 0xFF);
        a[4] = static_cast<char>(id & 0xFF);
        a[5] = 0x2B;
        a[6] = 0x40;
        a[7] = 0x60;
        a[8] = 0x00;
        a[9] = static_cast<char>(value);
        udp_->writeDatagram(a, serverIp_, serverPort_);
    }

    void writeU32(uint16_t id, uint8_t cmd, uint8_t idxL, uint8_t idxH, uint32_t data) {
        QByteArray a(13, 0x00);
        a[0] = 0x08;
        a[3] = static_cast<char>((id >> 8) & 0xFF);
        a[4] = static_cast<char>(id & 0xFF);
        a[5] = static_cast<char>(cmd);
        a[6] = static_cast<char>(idxL);
        a[7] = static_cast<char>(idxH);
        a[8] = 0x00;
        a[9] = static_cast<char>(data & 0xFF);
        a[10] = static_cast<char>((data >> 8) & 0xFF);
        a[11] = static_cast<char>((data >> 16) & 0xFF);
        a[12] = static_cast<char>((data >> 24) & 0xFF);
        udp_->writeDatagram(a, serverIp_, serverPort_);
    }

    void readPosition(uint16_t id) {
        QByteArray a(13, 0x00);
        a[0] = 0x08;
        a[3] = static_cast<char>((id >> 8) & 0xFF);
        a[4] = static_cast<char>(id & 0xFF);
        a[5] = 0x40;
        a[6] = 0x64;
        a[7] = 0x60;
        udp_->writeDatagram(a, serverIp_, serverPort_);
    }

    void readCurrent(uint16_t id) {
        QByteArray a(13, 0x00);
        a[0] = 0x08;
        a[3] = static_cast<char>((id >> 8) & 0xFF);
        a[4] = static_cast<char>(id & 0xFF);
        a[5] = 0x40;
        a[6] = 0x77;
        a[7] = 0x60;
        udp_->writeDatagram(a, serverIp_, serverPort_);
    }

    static int idToIndex(uint8_t idLsb) {
        switch (idLsb) {
        case 0x81: return 1;
        case 0x82: return 2;
        case 0x83: return 3;
        case 0x84: return 4;
        case 0x85: return 5;
        default: return 0;
        }
    }

    void disposeReadCurrent(const QByteArray& a) {
        bool ok = false;
        const int idx = idToIndex(static_cast<uint8_t>(a[4]));
        if (idx == 0) {
            return;
        }
        current_[idx] = static_cast<int16_t>(a.toHex().mid(18, 2).toInt(&ok, 16) |
                                             (a.toHex().mid(20, 2).toInt(&ok, 16) << 8));
    }

    void disposeReadPosition(const QByteArray& a) {
        bool ok = false;
        const int idx = idToIndex(static_cast<uint8_t>(a[4]));
        if (idx == 0) {
            return;
        }
        position_[idx] = static_cast<int32_t>(a.toHex().mid(18, 2).toInt(&ok, 16) |
                                              (a.toHex().mid(20, 2).toInt(&ok, 16) << 8) |
                                              (a.toHex().mid(22, 2).toInt(&ok, 16) << 16) |
                                              (a.toHex().mid(24, 2).toInt(&ok, 16) << 24));
    }

private:
    QUdpSocket* udp_ = nullptr;
    QHostAddress serverIp_;
    uint16_t serverPort_ = 0;
    QTimer* readTimer_ = nullptr;
    int readTimerNum_ = 0;

    std::array<int32_t, 6> position_ {0, 0, 0, 0, 0, 0};
    std::array<int16_t, 6> current_ {0, 0, 0, 0, 0, 0};
};

class EyouPPDriver final : public QObject {
public:
    explicit EyouPPDriver(QObject* parent = nullptr) : QObject(parent) {
        udp_ = new QUdpSocket(this);
        udp_->bind(7100);
        serverIp_.setAddress("192.168.1.253");
        serverPort_ = 1030;

        readTimer_ = new QTimer(this);
        readTimer_->setInterval(50);

        QObject::connect(readTimer_, &QTimer::timeout, this, &EyouPPDriver::readTimerFun);
        QObject::connect(udp_, &QUdpSocket::readyRead, this, &EyouPPDriver::receiveData);
    }

    void openReadTimer() { readTimer_->start(); }

    void setEnable(uint8_t id) { writeSimpleCommand(id, 0x01, 0x10, 0x00000001); }
    void setVelocityMode(uint8_t id) { writeSimpleCommand(id, 0x01, 0x0F, 0x00000003); }
    void setPositionMode(uint8_t id) { writeSimpleCommand(id, 0x01, 0x0F, 0x00000001); }

    // 按原项目写法，PP 速度写入顺序保持不变（高字节在前）。
    void setVelocity(uint8_t id, uint32_t data) { writeDataCommand(id, 0x01, 0x09, data, true); }
    void setPosition(uint8_t id, uint32_t data) { writeDataCommand(id, 0x01, 0x0A, data, true); }
    void setStop(uint8_t id) { setVelocity(id, 0); }
    void readEnable(uint8_t id) {
        QByteArray a(13, 0x00);
        a[0] = 0x06;
        a[4] = static_cast<char>(id);
        a[5] = 0x03;
        a[6] = 0x10;
        udp_->writeDatagram(a, serverIp_, serverPort_);
    }

    const std::array<int32_t, 8>& positions() const { return position_; }
    const std::array<bool, 8>& enables() const { return enable_; }

    void readTimerFun() {
        switch (readTimerNum_) {
        case 1: readPosition(static_cast<uint8_t>(Arm_Eyou_1)); break;
        case 3: readPosition(static_cast<uint8_t>(Arm_Eyou_3)); break;
        case 4: readPosition(static_cast<uint8_t>(Arm_Eyou_4)); break;
        case 5: readPosition(static_cast<uint8_t>(Arm_Eyou_5)); break;
        case 6:
            readPosition(static_cast<uint8_t>(Arm_Eyou_6));
            readTimerNum_ = 0;
            break;
        default:
            break;
        }
        ++readTimerNum_;
    }

    void receiveData() {
        QByteArray a;
        a.resize(static_cast<int>(udp_->bytesAvailable()));
        udp_->readDatagram(a.data(), a.size());
        if (a.size() < 13) {
            return;
        }

        bool ok = false;
        if (a[5] == static_cast<char>(0x02) && a[6] == static_cast<char>(0x10)) {
            const uint8_t id = static_cast<uint8_t>(a[4]);
            disposeEnable(id, a[7] != 0);
        } else if (a[5] == static_cast<char>(0x04) && a[6] == static_cast<char>(0x07)) {
            const uint8_t id = static_cast<uint8_t>(a[4]);
            const int32_t v = static_cast<int32_t>(a.toHex().mid(14, 8).toUInt(&ok, 16));
            disposeReadPosition(id, v);
        } else if (a[5] == static_cast<char>(0x04) && a[6] == static_cast<char>(0x10)) {
            const uint8_t id = static_cast<uint8_t>(a[4]);
            disposeEnable(id, a[10] != 0);
        }
    }

private:
    void writeSimpleCommand(uint8_t id, uint8_t rw, uint8_t subcmd, uint32_t data) {
        writeDataCommand(id, rw, subcmd, data, true);
    }

    void writeDataCommand(uint8_t id, uint8_t rw, uint8_t subcmd, uint32_t data, bool bigEndianPayload) {
        QByteArray a(13, 0x00);
        a[0] = 0x06;
        a[4] = static_cast<char>(id);
        a[5] = static_cast<char>(rw);
        a[6] = static_cast<char>(subcmd);

        if (bigEndianPayload) {
            a[7] = static_cast<char>((data >> 24) & 0xFF);
            a[8] = static_cast<char>((data >> 16) & 0xFF);
            a[9] = static_cast<char>((data >> 8) & 0xFF);
            a[10] = static_cast<char>(data & 0xFF);
        } else {
            a[7] = static_cast<char>(data & 0xFF);
            a[8] = static_cast<char>((data >> 8) & 0xFF);
            a[9] = static_cast<char>((data >> 16) & 0xFF);
            a[10] = static_cast<char>((data >> 24) & 0xFF);
        }

        udp_->writeDatagram(a, serverIp_, serverPort_);
    }

    void readPosition(uint8_t id) {
        QByteArray a(13, 0x00);
        a[0] = 0x06;
        a[4] = static_cast<char>(id);
        a[5] = 0x03;
        a[6] = 0x07;
        udp_->writeDatagram(a, serverIp_, serverPort_);
    }

    void disposeReadPosition(uint8_t id, int32_t result) {
        switch (id) {
        case static_cast<uint8_t>(Arm_Eyou_1): position_[1] = result; break;
        case static_cast<uint8_t>(Arm_Eyou_3): position_[3] = result; break;
        case static_cast<uint8_t>(Arm_Eyou_4): position_[4] = result; break;
        case static_cast<uint8_t>(Arm_Eyou_5): position_[5] = result; break;
        case static_cast<uint8_t>(Arm_Eyou_6): position_[6] = result; break;
        case static_cast<uint8_t>(Arm_Eyou_7): position_[7] = result; break;
        default: break;
        }
    }

    void disposeEnable(uint8_t id, bool result) {
        switch (id) {
        case static_cast<uint8_t>(Arm_Eyou_1): enable_[1] = result; break;
        case static_cast<uint8_t>(Arm_Eyou_3): enable_[3] = result; break;
        case static_cast<uint8_t>(Arm_Eyou_4): enable_[4] = result; break;
        case static_cast<uint8_t>(Arm_Eyou_5): enable_[5] = result; break;
        case static_cast<uint8_t>(Arm_Eyou_6): enable_[6] = result; break;
        case static_cast<uint8_t>(Arm_Eyou_7): enable_[7] = result; break;
        default: break;
        }
    }

private:
    QUdpSocket* udp_ = nullptr;
    QHostAddress serverIp_;
    uint16_t serverPort_ = 0;
    QTimer* readTimer_ = nullptr;
    int readTimerNum_ = 0;

    std::array<int32_t, 8> position_ {0, 0, 0, 0, 0, 0, 0, 0};
    std::array<bool, 8> enable_ {false, false, false, false, false, false, false, false};
};

class MTDriver final : public QObject {
public:
    explicit MTDriver(QObject* parent = nullptr) : QObject(parent) {
        udp_ = new QUdpSocket(this);
        udp_->bind(7000);
        serverIp_.setAddress("192.168.1.253");
        serverPort_ = 1030;

        readTimer_ = new QTimer(this);
        readTimer_->setInterval(80);

        QObject::connect(readTimer_, &QTimer::timeout, this, &MTDriver::readTimerFun);
        QObject::connect(udp_, &QUdpSocket::readyRead, this, &MTDriver::receiveData);
    }

    void openReadTimer() { readTimer_->start(); }

    void setVelocity(uint16_t id, int32_t data) {
        QByteArray a(13, 0x00);
        a[0] = 0x08;
        a[3] = static_cast<char>((id >> 8) & 0xFF);
        a[4] = static_cast<char>(id & 0xFF);
        a[5] = static_cast<char>(0xA2);
        a[9] = static_cast<char>(static_cast<uint8_t>(data));
        a[10] = static_cast<char>(static_cast<uint8_t>(data >> 8));
        a[11] = static_cast<char>(static_cast<uint8_t>(data >> 16));
        a[12] = static_cast<char>(static_cast<uint8_t>(data >> 24));
        udp_->writeDatagram(a, serverIp_, serverPort_);
    }

    void setPosition(uint16_t id, int32_t p, int32_t v) {
        QByteArray a(13, 0x00);
        a[0] = 0x08;
        a[3] = static_cast<char>((id >> 8) & 0xFF);
        a[4] = static_cast<char>(id & 0xFF);
        a[5] = static_cast<char>(0xA4);
        a[7] = static_cast<char>(static_cast<uint8_t>(v));
        a[8] = static_cast<char>(static_cast<uint8_t>(v >> 8));
        a[9] = static_cast<char>(static_cast<uint8_t>(p));
        a[10] = static_cast<char>(static_cast<uint8_t>(p >> 8));
        a[11] = static_cast<char>(static_cast<uint8_t>(p >> 16));
        a[12] = static_cast<char>(static_cast<uint8_t>(p >> 24));
        udp_->writeDatagram(a, serverIp_, serverPort_);
    }

    void setStop(uint16_t id) {
        QByteArray a(13, 0x00);
        a[0] = 0x08;
        a[3] = static_cast<char>((id >> 8) & 0xFF);
        a[4] = static_cast<char>(id & 0xFF);
        a[5] = static_cast<char>(0x81);
        udp_->writeDatagram(a, serverIp_, serverPort_);
    }

    void setAllStop() {
        QByteArray a(13, 0x00);
        a[0] = 0x08;
        a[3] = 0x02;
        a[4] = static_cast<char>(0x80);
        a[5] = static_cast<char>(0x81);
        udp_->writeDatagram(a, serverIp_, serverPort_);
    }

    void stop() {
        setAllStop();
        spinSleep(10);
        setAllStop();
        spinSleep(10);
        setAllStop();
    }

    void restart() {
        resetSystem(MT_Wheel_1);
        spinSleep(10);
        resetSystem(MT_Wheel_2);
        spinSleep(10);
    }

    void setNowToZero(uint16_t id) {
        QByteArray a(13, 0x00);
        a[0] = 0x08;
        a[3] = static_cast<char>((id >> 8) & 0xFF);
        a[4] = static_cast<char>(id & 0xFF);
        a[5] = 0x64;
        udp_->writeDatagram(a, serverIp_, serverPort_);
    }

    void Forward(int mode) {
        if (mode == 1) {
            setVelocity(MT_Wheel_1, MT_Wheel_1_DIR * MT_Wheel_1_Vel_LOW);
            spinSleep(20);
            setVelocity(MT_Wheel_2, -MT_Wheel_2_DIR * MT_Wheel_2_Vel_LOW);
        } else if (mode == 2) {
            setVelocity(MT_Wheel_1, MT_Wheel_1_DIR * MT_Wheel_2_Vel_HIGH);
            spinSleep(20);
            setVelocity(MT_Wheel_2, -MT_Wheel_2_DIR * MT_Wheel_2_Vel_HIGH);
        }
    }

    void Retreat(int mode) {
        if (mode == 1) {
            setVelocity(MT_Wheel_1, -MT_Wheel_1_DIR * MT_Wheel_1_Vel_LOW);
            spinSleep(20);
            setVelocity(MT_Wheel_2, MT_Wheel_2_DIR * MT_Wheel_2_Vel_LOW);
        } else if (mode == 2) {
            setVelocity(MT_Wheel_1, -MT_Wheel_1_DIR * MT_Wheel_2_Vel_HIGH);
            spinSleep(20);
            setVelocity(MT_Wheel_2, MT_Wheel_2_DIR * MT_Wheel_2_Vel_HIGH);
        }
    }

    void LeftSteering() {
        setVelocity(MT_Wheel_1, MT_Wheel_1_DIR * MT_Wheel_1_Vel_LEFT);
        spinSleep(10);
        setVelocity(MT_Wheel_2, MT_Wheel_2_DIR * MT_Wheel_2_Vel_LEFT);
    }

    void RightSteering() {
        setVelocity(MT_Wheel_1, -MT_Wheel_1_DIR * MT_Wheel_1_Vel_RIGHT);
        spinSleep(10);
        setVelocity(MT_Wheel_2, -MT_Wheel_2_DIR * MT_Wheel_2_Vel_RIGHT);
    }

    const std::array<double, 3>& currents() const { return current_; }

    void readTimerFun() {
        switch (readTimerNum_) {
        case 1: readState2(0x141); break;
        case 2:
            readState2(0x14B);
            readTimerNum_ = 0;
            break;
        default:
            break;
        }
        ++readTimerNum_;
    }

    void receiveData() {
        QByteArray a;
        a.resize(static_cast<int>(udp_->bytesAvailable()));
        udp_->readDatagram(a.data(), a.size());
        if (a.size() < 13) {
            return;
        }

        if (a[5] == static_cast<char>(0x9C)) {
            disposeState2(a);
        } else if (a[5] == static_cast<char>(0x64)) {
            resetSystem(static_cast<uint16_t>(static_cast<uint8_t>(a[4]) + 0x0100));
        }
    }

private:
    void resetSystem(uint16_t id) {
        QByteArray a(13, 0x00);
        a[0] = 0x08;
        a[3] = static_cast<char>((id >> 8) & 0xFF);
        a[4] = static_cast<char>(id & 0xFF);
        a[5] = 0x76;
        udp_->writeDatagram(a, serverIp_, serverPort_);
    }

    void readState2(uint16_t id) {
        QByteArray a(13, 0x00);
        a[0] = 0x08;
        a[3] = static_cast<char>((id >> 8) & 0xFF);
        a[4] = static_cast<char>(id & 0xFF);
        a[5] = static_cast<char>(0x9C);
        udp_->writeDatagram(a, serverIp_, serverPort_);
    }

    void readMultiTurnAngle(uint16_t id) {
        QByteArray a(13, 0x00);
        a[0] = 0x08;
        a[3] = static_cast<char>((id >> 8) & 0xFF);
        a[4] = static_cast<char>(id & 0xFF);
        a[5] = 0x60;
        udp_->writeDatagram(a, serverIp_, serverPort_);
    }

    void disposeState2(const QByteArray& a) {
        bool ok = false;
        const int16_t t = static_cast<int16_t>(a.toHex().mid(14, 2).toInt(&ok, 16) +
                                               (a.toHex().mid(16, 2).toInt(&ok, 16) * 256));
        if (a[4] == static_cast<char>(0x41)) {
            current_[0] = static_cast<double>(t) / 100.0;
        } else if (a[4] == static_cast<char>(0x4B)) {
            current_[1] = static_cast<double>(t) / 100.0;
        }
    }

private:
    QUdpSocket* udp_ = nullptr;
    QHostAddress serverIp_;
    uint16_t serverPort_ = 0;
    QTimer* readTimer_ = nullptr;
    int readTimerNum_ = 0;

    std::array<double, 3> current_ {0.0, 0.0, 0.0};
};

// 用于复用原工程初始化顺序的总控封装。
class MotorDriverPack final {
public:
    explicit MotorDriverPack(QObject* parent = nullptr)
        : mt(parent), pp(parent), ph(parent) {}

    void initializeAll() {
        mt.openReadTimer();
        pp.openReadTimer();
        ph.openReadTimer();

        spinSleep(50);
        mt.setVelocity(MT_Wheel_1, 0);
        spinSleep(50);
        mt.setVelocity(MT_Wheel_2, 0);
        spinSleep(50);

        ph.setVelocityMode(Arm_Eyou_2);
        spinSleep(50);
        ph.setAcceleration(Arm_Eyou_2, static_cast<uint32_t>(1e7));
        spinSleep(50);
        ph.setDeceleration(Arm_Eyou_2, static_cast<uint32_t>(1e7));
        spinSleep(50);
        ph.setShutdown06(Arm_Eyou_2);
        spinSleep(50);
        ph.setSwitchOn07(Arm_Eyou_2);
        spinSleep(50);

        initSinglePH(swingArm1);
        initSinglePH(swingArm2);
        initSinglePH(swingArm3);
        initSinglePH(swingArm4);

        initSinglePP(static_cast<uint8_t>(Arm_Eyou_1));
        initSinglePP(static_cast<uint8_t>(Arm_Eyou_3));
        initSinglePP(static_cast<uint8_t>(Arm_Eyou_4));
        initSinglePP(static_cast<uint8_t>(Arm_Eyou_5));
        initSinglePP(static_cast<uint8_t>(Arm_Eyou_6));
        initSinglePP(static_cast<uint8_t>(Arm_Eyou_7));
    }

public:
    MTDriver mt;
    EyouPPDriver pp;
    EyouPHDriver ph;

private:
    void initSinglePH(uint16_t id) {
        ph.setVelocityMode(id);
        spinSleep(50);
        ph.setAcceleration(id, static_cast<uint32_t>(1e7));
        spinSleep(50);
        ph.setDeceleration(id, static_cast<uint32_t>(1e7));
        spinSleep(50);
        ph.setShutdown06(id);
        spinSleep(50);
        ph.setSwitchOn07(id);
        spinSleep(50);
    }

    void initSinglePP(uint8_t id) {
        pp.setEnable(id);
        spinSleep(50);
        pp.setVelocityMode(id);
        spinSleep(50);
    }
};

} // namespace rebot_motor

#endif // MOTOR_DRIVER_SINGLE_FILE_HPP
