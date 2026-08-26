#include "I2cDevice.h"

#include <cerrno>    // errno
#include <cstring>   // std::strerror()
#include <stdexcept> // std::runtime_error, std::invalid_argument

#include <fcntl.h>         // open(), O_RDWR, O_CLOEXEC
#include <linux/i2c-dev.h> // I2C_SLAVE, I2C_RDWR, i2c_rdwr_ioctl_data
#include <linux/i2c.h>     // i2c_msg, I2C_M_RD
#include <sys/ioctl.h>     // ioctl()
#include <unistd.h>        // close(), read(), write()

// I2cDevice는 Linux의 I2C 디바이스 파일을 통해 실제 I2C 장치와 통신하는 저수준 하드웨어 통신 클래스입니다.
// 상위의 PWM Controller나 IMU Controller가 레지스터 단위로 데이터를 읽고 쓸 수 있도록 공통 I2C Read/Write 인터페이스를 제공합니다.

// IMU나 PWM Controller에서 open, ioctl, read/write 같은 저수준 Linux I2C 코드를 반복하지 않고,
// 공통 통신 기능을 I2cDevice로 분리해서 재사용하기 위해서입니다.
I2cDevice::I2cDevice(const std::string &device, uint8_t address) : address_(address)
{
    // Linux I2C 장치 파일을 읽기/쓰기 모드로 열고 file descriptor를 fd_에 저장
    // device는 std::string이고 open()은 const char*를 받으므로 c_str() 사용
    fd_ = open(device.c_str(), O_RDWR | O_CLOEXEC);

    if (fd_ < 0)
        throw std::runtime_error("Failed to open I2C device");

    // 이후 read()/write() 시스템 콜이 통신할 I2C Slave 주소 설정
    // I2C_SLAVE는 해당 file descriptor의 기본 slave 주소를 지정하는 ioctl 명령
    if (ioctl(fd_, I2C_SLAVE, address) < 0)
    {
        close(fd_); // 설정 실패 시 열었던 I2C 장치 닫기
        fd_ = -1;   // 유효한 파일 디스크립터가 없음을 표시
        throw std::runtime_error("Failed to select I2C slave");
    }
}

I2cDevice::~I2cDevice()
{
    if (fd_ >= 0)
        close(fd_);
}

void I2cDevice::writeRegister16(uint8_t reg, uint16_t value)
{
    uint8_t data[3];

    // 장치가 MSB → LSB 순서를 요구하므로 상위 바이트부터 저장
    data[0] = reg;                                       // 어느 레지스터에 쓸지
    data[1] = static_cast<uint8_t>((value >> 8) & 0xff); // 상위 8bit
    data[2] = static_cast<uint8_t>(value & 0xff);        // 하위 8bit

    // data의 3바이트를 I2C 장치로 전송하고, 전부 전송되지 않으면 예외 발생
    if (write(fd_, data, sizeof(data)) != static_cast<ssize_t>(sizeof(data)))
    // static_cast<ssize_t>(sizeof(data)) = sizeof(data)의 결과를
    // write() 반환 타입인 ssize_t로 맞춰주는 명시적 형 변환.
    {
        throw std::runtime_error(std::string("I2C write failed: ") + std::strerror(errno));
    }
}

void I2cDevice::writeRegister8(uint8_t reg, uint8_t value)
{
    uint8_t data[2];

    data[0] = reg;
    data[1] = value;

    if (write(fd_, data, sizeof(data)) != static_cast<ssize_t>(sizeof(data)))
    {
        throw std::runtime_error(std::string("I2C write failed: ") + std::strerror(errno));
    }
}

// readRegister8(): 1바이트 읽기용 편의 함수
uint8_t I2cDevice::readRegister8(uint8_t reg)
{
    uint8_t value = 0;
    readRegisters(reg, &value, 1);
    return value;
}

// readRegisters() → 실제 읽기를 수행하는 범용 함수. length를 받아서 몇 바이트 읽을지 결정
void I2cDevice::readRegisters(uint8_t startReg, uint8_t *buffer, std::size_t length)
// startReg -> 읽기 시작할 레지스터 주소, buffer -> 읽은 데이터를 저장할 메모리, length -> 몇 바이트 읽을지
{
    if (buffer == nullptr || length == 0)
        throw std::invalid_argument("I2C read buffer must be non-null and length > 0");

    // I2C 레지스터 읽기:
    // 1. 읽기 시작할 레지스터 주소를 Write
    // 2. 해당 레지스터부터 데이터를 Read
    //
    // 두 동작을 하나의 combined transaction으로 처리하기 위해 i2c_msg 2개 사용.
    // i2c_msg 하나는 하나의 I2C 메시지(Write 또는 Read)를 나타냄.
    i2c_msg messages[2]{};

    // 첫 번째 메시지: 시작 레지스터 주소 전송
    messages[0].addr = address_;
    messages[0].flags = 0; // I2C_M_RD가 없으므로 Write
    messages[0].len = 1;
    messages[0].buf = &startReg;

    // 두 번째 메시지: 실제 데이터 읽기
    messages[1].addr = address_;
    messages[1].flags = I2C_M_RD;
    messages[1].len = static_cast<__u16>(length);
    messages[1].buf = buffer;

    // 여러 i2c_msg를 하나의 combined transaction으로 묶음
    i2c_rdwr_ioctl_data transaction{};

    transaction.msgs = messages;
    transaction.nmsgs = 2;

    // START → Slave+Write → startReg
    // → Repeated START → Slave+Read → Data → STOP
    if (ioctl(fd_, I2C_RDWR, &transaction) < 0)
    {
        throw std::runtime_error(std::string("I2C combined read failed: ") + std::strerror(errno));
    }
}