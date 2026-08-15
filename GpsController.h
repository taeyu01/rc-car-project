#pragma once

#include <iostream>
#include <string>
#include <sstream>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <stdexcept>
#include <opencv2/opencv.hpp>
#include <atomic>     // isThreadRun_을 여러 Thread가 안전하게 읽고 쓰기 위해 사용
#include <functional> // GPS 갱신 후 지도 갱신 callback을 받기 위해 사용

class SeoilCoordController;
class RcCarDataManager;

class GpsController
{
public:
    // RcCarDataManager를 생성할 때 전달받아
    // GPS 위치 데이터를 저장할 수 있도록 연결
    GpsController(const char *serialPort, RcCarDataManager &dataManager);

    ~GpsController();

    // GPS 데이터를 계속 수신/파싱하고 좌표 변환 후 저장.
    // GPS 위치가 새로 저장될 때마다 onGpsUpdated()를 실행해서
    // GPS Thread에서 지도 갱신까지 이어서 수행.
    void runGpsThread(const SeoilCoordController &coordController, const std::function<void()> &onGpsUpdated);

    // GPS Thread 종료 요청
    void stopThread();

private:
    // UART로부터 데이터를 읽고
    // 완성된 NMEA($GPGGA) 문장에서 위도/경도를 가져오는 함수
    bool getGpsData(double &lat, double &lon);

    // GPGGA 문장에서 위도/경도 원본 문자열 추출
    bool parseGpsData(const std::string &nmeaLine, double &lat, double &lon);

    // NMEA DDMM.MMMM 형식을 Decimal Degrees로 변환
    bool convertToDegree(const std::string &degreeText, double &degrees);

    // UART 파일 디스크립터
    int uartFilestream_ = -1;

    // 아직 완성되지 않은 UART 데이터를 임시 저장
    std::string rxBuffer_;

    // GPS Thread가 읽고 다른 Thread가 stopThread()에서 수정하므로
    // 일반 bool 대신 atomic 사용
    std::atomic<bool> isThreadRun_{true};

    // main에서 생성한 RcCarDataManager를 복사하지 않고 참조
    RcCarDataManager &dataManager_;
};