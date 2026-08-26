#include "ImuController.h"

#include <chrono>
#include <stdexcept>
#include <thread>

namespace
{
    // BNO055 내부의 칩 ID가 저장되어 있는 레지스터 주소
    constexpr uint8_t CHIP_ID_REG = 0x00;

    // BNO055의 고유 칩 ID 값
    // CHIP_ID_REG를 읽었을 때 이 값이 나오면 BNO055가 정상적으로 연결된 것으로 판단
    constexpr uint8_t BNO055_CHIP_ID = 0xA0;

    // BNO055의 동작 모드를 설정하는 레지스터 주소
    constexpr uint8_t OPR_MODE_REG = 0x3D;

    // 리셋 등의 시스템 동작을 설정하는 레지스터 주소
    constexpr uint8_t SYS_TRIGGER_REG = 0x3F;

    // Euler 각도 데이터가 시작되는 레지스터 주소
    // 0x1A부터 Heading, Roll, Pitch 데이터가 각각 2바이트씩 저장되어 있음
    constexpr uint8_t EULER_H_LSB = 0x1A;

    // BNO055의 설정을 변경하기 위한 CONFIG 모드 값
    constexpr uint8_t CONFIG_MODE = 0x00;

    // 가속도계 + 자이로스코프 + 지자기 센서의 값을 융합해서
    // 방향과 자세를 계산하는 NDOF 모드 값
    constexpr uint8_t NDOF_MODE = 0x0C;

    // BNO055의 Euler 원시값은 실제 각도의 16배로 저장되므로
    // 실제 각도(°)로 변환하기 위해 사용하는 값
    constexpr double EULER_SCALE = 16.0;
}

// 외부에서 생성한 BNO055용 I2cDevice 객체를 참조로 받아 i2c_에 연결
// 실제 BNO055의 I2C 주소(0x28)는 I2cDevice 객체를 생성할 때 설정
ImuController::ImuController(I2cDevice &i2c) : i2c_(i2c)
{
}

void ImuController::initialize()
{
    // BNO055의 CHIP_ID_REG(0x00)를 읽어 현재 연결된 장치의 칩 ID 확인
    uint8_t chipId = i2c_.readRegister8(CHIP_ID_REG);

    // 읽어온 칩 ID가 BNO055의 고유 ID인 0xA0이 아니라면
    // BNO055가 정상적으로 연결되지 않은 것으로 판단하고 예외 발생
    if (chipId != BNO055_CHIP_ID)
        throw std::runtime_error("BNO055 not detected");

    // BNO055의 설정을 변경하기 위해 OPR_MODE_REG에
    // CONFIG_MODE(0x00)를 써서 CONFIG 모드로 전환
    i2c_.writeRegister8(OPR_MODE_REG, CONFIG_MODE);

    // 센서가 CONFIG 모드로 전환될 시간을 주기 위해 25ms 대기
    std::this_thread::sleep_for(std::chrono::milliseconds(25));

    // SYS_TRIGGER_REG에 0x00을 써서 일반 동작 상태로 설정
    i2c_.writeRegister8(SYS_TRIGGER_REG, 0x00);

    // 설정이 적용될 시간을 주기 위해 10ms 대기
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // OPR_MODE_REG에 NDOF_MODE(0x0C)를 써서
    // 가속도계 + 자이로스코프 + 지자기 센서를 융합하는 NDOF 모드로 전환
    i2c_.writeRegister8(OPR_MODE_REG, NDOF_MODE);

    // 센서가 NDOF 모드로 전환될 시간을 주기 위해 25ms 대기
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
}

int16_t ImuController::combineBytes(uint8_t low, uint8_t high)
{
    // BNO055의 각도 데이터 하나는 16비트이지만
    // 레지스터에는 LSB와 MSB 각각 8비트씩 나누어 저장되어 있음

    // high(MSB)를 왼쪽으로 8비트 이동시켜 상위 8비트에 배치하고
    // low(LSB)와 OR 연산하여 하나의 16비트 값으로 결합
    uint16_t value =
        static_cast<uint16_t>(low) |
        (static_cast<uint16_t>(high) << 8);

    // Roll, Pitch 값은 음수가 될 수 있으므로
    // 부호가 있는 16비트 정수인 int16_t로 변환하여 반환
    return static_cast<int16_t>(value);
}

ImuData ImuController::read()
{
    // Heading, Roll, Pitch는 각각 2바이트이므로
    // 총 6바이트의 데이터를 저장할 배열 생성
    uint8_t data[6];

    // Euler 데이터가 시작되는 0x1A 레지스터부터 연속으로 6바이트 읽기
    // data[0], data[1] : Heading LSB, MSB
    // data[2], data[3] : Roll LSB, MSB
    // data[4], data[5] : Pitch LSB, MSB
    i2c_.readRegisters(EULER_H_LSB, data, sizeof(data));

    // 각각의 LSB와 MSB를 결합하여 16비트 Heading 원시값 생성
    int16_t rawHeading = combineBytes(data[0], data[1]);

    // Roll의 LSB와 MSB를 결합하여 16비트 Roll 원시값 생성
    int16_t rawRoll = combineBytes(data[2], data[3]);

    // Pitch의 LSB와 MSB를 결합하여 16비트 Pitch 원시값 생성
    int16_t rawPitch = combineBytes(data[4], data[5]);

    // 변환한 Heading, Roll, Pitch 값을 하나로 묶어서 반환하기 위한 구조체
    ImuData result;

    // BNO055의 Euler 원시값은 실제 각도의 16배이므로
    // 각각 16으로 나누어 실제 각도(°)로 변환
    result.heading = rawHeading / EULER_SCALE;
    result.roll = rawRoll / EULER_SCALE;
    result.pitch = rawPitch / EULER_SCALE;

    // Heading, Roll, Pitch가 저장된 ImuData 구조체 반환
    return result;
}