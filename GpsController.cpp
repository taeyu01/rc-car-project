#include "GpsController.h"
#include "SeoilCoordController.h"
#include "RcCarDataManager.h"

GpsController::GpsController(const char *serialPort, RcCarDataManager &dataManager) : dataManager_(dataManager)
{
    uartFilestream_ = open(serialPort, O_RDONLY | O_NOCTTY);

    if (uartFilestream_ < 0)
    {
        throw std::runtime_error(
            std::string("Failed to open ") + serialPort);
    }

    // 기존에 커널 UART 입력 버퍼에 남아있던 데이터 제거
    tcflush(uartFilestream_, TCIFLUSH);

    struct termios options;

    // UART 현재 설정을 가져오지 못하면
    // 잘못된 설정값을 사용하지 않도록 예외 처리
    if (tcgetattr(uartFilestream_, &options) < 0)
    {
        close(uartFilestream_);
        uartFilestream_ = -1;

        throw std::runtime_error("Failed to get GPS UART configuration");
    }

    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);

    options.c_cflag &= ~PARENB;        // Parity 사용 안 함
    options.c_cflag &= ~CSTOPB;        // Stop Bit 1개 사용
    options.c_cflag &= ~CSIZE;         // 데이터 비트 설정 초기화
    options.c_cflag |= CS8;            // 8bit 데이터
    options.c_cflag &= ~CRTSCTS;       // RTS/CTS 비활성화
    options.c_cflag |= CREAD | CLOCAL; // Receiver 활성화

    // Canonical / Echo / Signal 처리 비활성화
    // → UART 데이터를 들어오는 즉시 바이트 단위로 처리
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    // 출력 데이터 후처리 비활성화
    options.c_oflag &= ~OPOST;

    // GPS 데이터가 없더라도 read()가 무한정 기다리지 않게 함.
    //
    // VMIN = 0
    // VTIME = 1
    // → 최대 약 0.1초 후 read() 반환
    //
    // 따라서 stopThread() 호출 여부를 주기적으로 확인 가능
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 1;

    if (tcsetattr(uartFilestream_, TCSANOW, &options) < 0)
    {
        close(uartFilestream_);
        uartFilestream_ = -1;

        throw std::runtime_error("Failed to configure GPS UART");
    }
}

GpsController::~GpsController()
{
    if (uartFilestream_ >= 0)
        close(uartFilestream_);
}

void GpsController::runGpsThread(const SeoilCoordController &coordController, const std::function<void()> &onGpsUpdated)
{
    while (isThreadRun_)
    {
        double lat = 0.0;
        double lon = 0.0;

        if (!getGpsData(lat, lon))
            continue;

        // GPS 위도/경도 → 지도 Pixel 좌표
        cv::Point2f pixel = coordController.getRcCarPixel(lat, lon);

        // GPS 위치와 Pixel 좌표를 DataManager에 저장
        dataManager_.updateGps(lat, lon, pixel.x, pixel.y);

        // GPS 위치가 새로 저장된 직후
        // main에서 전달한 지도 갱신 작업 실행
        //
        // 기존 Map Thread의 기능이 여기서 실행됨
        onGpsUpdated();
    }

    return;
}

void GpsController::stopThread()
{
    isThreadRun_ = false;
}

bool GpsController::getGpsData(double &lat, double &lon)
{
    char buffer[256];

    // GPS Thread 종료 요청이 들어오면
    // getGpsData() 내부에서도 빠져나올 수 있도록 함
    while (isThreadRun_)
    {
        size_t pos = rxBuffer_.find('\n');

        if (pos == std::string::npos)
        {
            int rx_length = read(uartFilestream_, (void *)buffer, sizeof(buffer) - 1);

            // UART timeout
            // 현재 데이터가 없으므로 runGpsThread()로 돌아가
            // 종료 여부를 다시 확인
            if (rx_length == 0)
                return false;

            // 실제 read() 오류와 timeout을 구분
            if (rx_length < 0)
            {
                // Signal 때문에 read()가 잠깐 중단된 경우 다시 시도
                if (errno == EINTR)
                    continue;

                throw std::runtime_error("GPS UART read failed");
            }

            rxBuffer_.append(buffer, rx_length);
            continue;
        }

        std::string nmeaLine = rxBuffer_.substr(0, pos);

        rxBuffer_.erase(0, pos + 1);

        // GPS 문장 끝의 '\r' 제거
        if (!nmeaLine.empty() && nmeaLine.back() == '\r')
        {
            nmeaLine.pop_back();
        }

        // GPGGA 문장만 사용
        if (nmeaLine.rfind("$GPGGA", 0) != 0)
            continue;

        if (parseGpsData(nmeaLine, lat, lon))
        {
            return true;
        }
    }

    return false;
}

bool GpsController::parseGpsData(const std::string &nmeaLine, double &lat, double &lon)
{
    std::stringstream stream(nmeaLine);

    std::string token;
    int index = 0;

    std::string lat_str;
    std::string lon_str;

    while (std::getline(stream, token, ','))
    {
        if (index == 2)
            lat_str = token;

        if (index == 4)
            lon_str = token;

        index++;
    }

    if (lat_str.empty() || lon_str.empty())
    {
        return false;
    }

    if (convertToDegree(lat_str, lat) && convertToDegree(lon_str, lon))
    {
        return true;
    }

    return false;
}

bool GpsController::convertToDegree(const std::string &degreeText, double &degrees)
{
    int divisor = 100;
    try
    {
        double rawDegree = std::stod(degreeText);

        int convertedDegrees = static_cast<int>(rawDegree / divisor);

        double minutes = rawDegree - (convertedDegrees * divisor);

        degrees = convertedDegrees + (minutes / 60.0);

        return true;
    }
    catch (...)
    {
        return false;
    }
}