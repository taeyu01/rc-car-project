#include "Camera.h"

#include <iostream>
#include <sstream>
#include <stdexcept>

Camera::Camera(int width, int height, int fps)
{
    // 일단 GStreamer 방식으로 카메라를 엶
    capture_.open(makePipeline(width, height, fps), cv::CAP_GSTREAMER);

    if (!capture_.isOpened())
    {
        std::cerr << "GStreamer camera open failed. Trying V4L2.\n";

        // GStreamer 실패하면 V4L2 방식으로 한번 더 엶
        capture_.open(0, cv::CAP_V4L2);
    }

    if (!capture_.isOpened())
        throw std::runtime_error("Failed to open camera");
}

Camera::~Camera()
{
    if (capture_.isOpened())

        // OpenCV의 VideoCapture가 사용 중인 카메라 자원을 해제
        capture_.release();
}

bool Camera::read(cv::Mat &frame)
{
    // VideoCapture의 read()를 호출하면
    // 카메라에서 다음 프레임을 가져와 frame에 저장
    return capture_.read(frame);
}

std::string Camera::makePipeline(int width, int height, int fps) const
{
    // 문자열을 조립하기 위한 객체
    std::ostringstream pipeline;

    pipeline
        // libcamera를 통해 카메라 영상을 가져오는 소스
        << "libcamerasrc ! "

        // 해상도 / 포맷 / FPS 설정
        << "video/x-raw,width=" << width
        << ",height=" << height
        << ",format=NV12,framerate=" << fps
        << "/1 ! "

        // 영상의 픽셀 포맷을 변환하는 GStreamer 요소
        << "videoconvert ! "

        // OpenCV에서 사용하기 위해 영상 포맷을 BGR로 지정
        << "video/x-raw,format=BGR ! "

        // 프레임을 임시 저장하는 queue를 생성하고 최대 1개의 프레임만 저장
        << "queue max-size-buffers=1 "

        // queue가 가득 차면 오래된 프레임을 버려 지연이 증가하는 것을 방지
        << "leaky=downstream ! "

        // GStreamer의 영상을 프로그램(OpenCV)으로 전달하고
        // 버퍼가 가득 찰 경우 오래된 프레임을 버림
        << "appsink drop=true "

        // appsink에 최대 1개의 프레임만 저장하여 프레임이 쌓이는 것을 방지
        << "max-buffers=1 "

        // 프레임의 타임스탬프에 맞춰 기다리지 않고 바로 전달하여 지연을 줄임
        << "sync=false";

    // 완성된 GStreamer 파이프라인을 std::string으로 변환하여 반환
    return pipeline.str();
}