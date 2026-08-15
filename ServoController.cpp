#include "ServoController.h"
#include "PwmController.h"

#include <algorithm> // std::clamp
#include <cmath>     // std::round
#include <cstdint>   // uint16_t
#include <stdexcept> // std::out_of_range

// ServoController 생성자
// 외부에서 만든 PwmController 객체를 참조해서 사용한다.
ServoController::ServoController(PwmController &pwm)
    : pwm_(pwm)
{
    // 서보 제어에 사용할 PWM Timer 설정
    pwm_.configureServoTimer();
}

// 각 서보 채널의 보정값(Calibration)을 설정
// channel 0 : Camera Pan
// channel 1 : Camera Tilt
// channel 2 : Steering
void ServoController::setCalibration(int channel, const ServoCalibration &calibration)
{
    // ServoController에서 사용하는 채널은 0~2만 허용
    checkChannel(channel);

    // 해당 채널의 각도 범위, 중심 보정값, 방향 반전 여부,
    // 최소/최대 PWM pulse width 등을 저장
    calibration_.at(channel) = calibration;
}

// 원하는 서보 각도를 실제 PWM 값으로 변환해서 출력
void ServoController::setAngle(int channel, double angle)
{
    checkChannel(channel);

    // 해당 채널에 저장된 Calibration 정보 가져오기
    // &를 사용하므로 복사하지 않고 기존 객체를 참조
    const auto &c = calibration_.at(channel);

    // 요청 각도를 해당 서보의 허용 범위로 제한
    // 예: Steering이 -30~30도라면 50도를 요청해도 30도로 제한
    angle = std::clamp(angle, c.minAngle, c.maxAngle);

    // 서보가 반대로 장착된 경우 각도 방향을 반전
    // reversed == true : +angle ↔ -angle
    double physicalAngle = c.reversed ? -angle : angle;

    // 실제 서보의 중앙 위치가 어긋난 경우 보정값 적용
    physicalAngle += c.centerOffset;

    // 최종 물리 각도를 -90~90도로 제한
    physicalAngle = std::clamp(physicalAngle, -90.0, 90.0);

    // 각도(-90~90도)를 서보 PWM pulse width로 변환
    //
    // 기본 설정이 500~2500us라면:
    // -90도 →  500us
    //   0도 → 1500us
    // +90도 → 2500us
    double pulseUs =
        c.minPulseUs + (physicalAngle + 90.0) * (c.maxPulseUs - c.minPulseUs) / 180.0;

    // pulse width(us)를 PWM 하드웨어의 카운트 값으로 변환
    //
    // 서보 PWM 한 주기 = 20,000us (20ms)
    // PWM counter 범위 = 0~4095
    //
    // 예: 1500us → 약 307
    uint16_t pwmValue = static_cast<uint16_t>(
        std::round(pulseUs * 4095.0 / 20000.0));

    // 계산된 PWM 값을 해당 PWM 채널에 실제로 설정
    // 이후 PwmController → I2cDevice → Linux → PWM HW로 내려감
    pwm_.setChannelValue(channel, pwmValue);
}

// 해당 서보를 논리적 중앙(0도)으로 이동
void ServoController::center(int channel)
{
    setAngle(channel, 0.0);
}

// 유효한 Servo 채널인지 검사
void ServoController::checkChannel(int channel)
{
    if (channel < 0 || channel > 2)
        throw std::out_of_range(
            "Servo channel must be 0~2");
}