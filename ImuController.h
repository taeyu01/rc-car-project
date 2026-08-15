#pragma once

#include "I2cDevice.h"

struct ImuData
{
    double heading;
    double roll;
    double pitch;
};

class ImuController
{
public:
    explicit ImuController(I2cDevice& i2c);

    void initialize();
    ImuData read();

private:
    I2cDevice& i2c_;

    static int16_t combineBytes(uint8_t low, uint8_t high);
};