#include "PwmController.h"

#include "I2cDevice.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

// Servo나 Motor가 원하는 출력을 PWM 칩의 레지스터 값으로 바꿔서 I2C로 써주는 클래스

// i2c_는 I2cDevice 타입 객체에 대한 참조를 저장하는 참조 멤버 변수이며
// 생성자에서 전달받은 I2cDevice 객체를 참조한다.
PwmController::PwmController(I2cDevice &i2c) : i2c_(i2c) {}

void PwmController::configureServoTimer()
{
    // (timer, prescaler, period);
    setTimer(0, 351, 4095);
}

void PwmController::configureMotorTimer()
{
    setTimer(3, 847, 849);
}

void PwmController::setChannelValue(int channel, uint16_t value)
{
    if (channel < 0 || channel > 19)
        throw std::out_of_range("PWM channel must be 0~19");

    int timer = getTimer(channel);

    // 타이머마다 주기가 다를 수 있으니까, 해당 채널이 사용하는 타이머의
    // period 값을 가져와서 지역 변수 period에 넣는 것
    uint16_t period = periods_[timer];

    if (period == 0)
        throw std::runtime_error("PWM timer not configured");

    // value가 period보다 커지지 못하게 제한하는 역할
    value = std::min(value, period);

    i2c_.writeRegister16(static_cast<uint8_t>(0x20 + channel), value);
}

void PwmController::setDutyPercent(int channel, double percent)
{
    if (channel < 0 || channel > 19)
        throw std::out_of_range("PWM channel must be 0~19");

    percent = std::clamp(percent, 0.0, 100.0);

    int timer = getTimer(channel);
    uint16_t period = periods_[timer];

    if (period == 0)
        throw std::runtime_error("PWM timer not configured");

    uint16_t value = static_cast<uint16_t>(std::round(period * percent / 100.0));
    setChannelValue(channel, value);
}

void PwmController::setTimer(int timer, uint16_t prescaler, uint16_t period)
{
    if (timer < 0 || timer > 6)
        throw std::out_of_range("Invalid PWM timer");

    uint8_t prescalerReg;
    uint8_t periodReg;

    // 데이터시트 참조
    if (timer < 4)
    {
        prescalerReg = 0x40 + timer;
        periodReg = 0x44 + timer;
    }
    else
    {
        prescalerReg = 0x50 + timer - 4;
        periodReg = 0x54 + timer - 4;
    }

    i2c_.writeRegister16(periodReg, period);
    i2c_.writeRegister16(prescalerReg, prescaler);

    periods_[timer] = period;
}

// PWM 채널 번호를 입력받아서, 그 채널이 공유하는 하드웨어 Timer 번호를 반환
int PwmController::getTimer(int channel) const
{
    if (channel < 16)
        return channel / 4;

    if (channel <= 17)
        return 4;

    if (channel == 18)
        return 5;

    return 6;
}