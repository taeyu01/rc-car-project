#include "GpioOutput.h"

#include <cstring>
#include <stdexcept>

#include <fcntl.h>
#include <linux/gpio.h>
#include <sys/ioctl.h>
#include <unistd.h>

GpioOutput::GpioOutput(unsigned int offset, const std::string &consumer)
{
    // GPIO 컨트롤러를 사용하기 위해 Linux가 제공하는 GPIO 장치 파일을 열겠다
    chipFd_ = open("/dev/gpiochip0", O_RDONLY | O_CLOEXEC);

    if (chipFd_ < 0)
        throw std::runtime_error("Failed to open GPIO chip");

    // 어떤 gpio 핀을 어떤 설정으로
    // 사용할지 커널에 요청하기 위한 구조체를 하나 만드는 코드
    gpio_v2_line_request request{};

    request.offsets[0] = offset;                     // 어떤 GPIO 핀을 사용할 것인지 지정
    request.num_lines = 1;                           // GPIO line을 몇 개 사용할 것인지 지정
    request.config.flags = GPIO_V2_LINE_FLAG_OUTPUT; // 선택한 GPIO를 출력 모드로 사용

    strncpy(request.consumer, consumer.c_str(), sizeof(request.consumer) - 1);

    // 안전하게 마지막 한 칸을 '\0' 자리로 확보
    request.consumer[sizeof(request.consumer) - 1] = '\0';

    if (ioctl(chipFd_, GPIO_V2_GET_LINE_IOCTL, &request) < 0)
    // chipFd_ -> 어떤 GPIO Controller 인지
    // GPIO_V2_GET_LINE_IOCTL -> Line 하나 요청
    // &request -> GPIO 번호, OUTPUT 설정 등이 들어있는 구조체
    {
        close(chipFd_);
        chipFd_ = -1;
        throw std::runtime_error("GPIO request failed");
    }
    // GPIO를 조작할 수 있는 전용 FD를 저장
    lineFd_ = request.fd;

    // GPIO를 바로 LOW로 만듦
    // 의도치 않은 하드웨어 동작 방지
    set(false);
}

GpioOutput::~GpioOutput()
{
    if (lineFd_ >= 0)
        close(lineFd_);

    if (chipFd_ >= 0)
        close(chipFd_);
}

void GpioOutput::set(bool value)
{
    // 출력할 값을 담는 구조체
    gpio_v2_line_values values{};

    // 어떤 GPIO를 변경할지 표시
    // 첫 번째 GPIO를 변경
    values.mask = 1;

    // value == true -> bits = 1 (HIGH)
    // value == false -> bits = 0 (LOW)
    values.bits = value ? 1 : 0;

    if (ioctl(lineFd_, GPIO_V2_LINE_SET_VALUES_IOCTL, &values) < 0)
        throw std::runtime_error("GPIO set failed");
}