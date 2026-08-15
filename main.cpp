#include <atomic>
#include <chrono>
#include <iostream>
#include <mutex>
#include <thread>
#include <opencv2/opencv.hpp>

#include "Camera.h"
#include "GpsController.h"
#include "I2cDevice.h"
#include "ImuController.h"
#include "MotorController.h"
#include "PwmController.h"
#include "RcCarDataManager.h"
#include "SeoilCoordController.h"
#include "ServoController.h"
#include "TerminalInput.h"

struct ControlCommand
{
    double speed = 0.0;         // 모터 속도
    double steeringAngle = 0.0; // RC카 조향각
    double cameraPan = 0.0;     // 카메라 좌우 회전
    double cameraTilt = 0.0;    // 카메라 상하 회전
};

int main()
{
    try
    {
        // ================= HARDWARE / CONTROLLER 생성 =================
        I2cDevice pwmI2c("/dev/i2c-1", 0x14);
        I2cDevice imuI2c("/dev/i2c-1", 0x28);

        PwmController pwm(pwmI2c);
        ServoController servo(pwm);
        MotorController motor(pwm);
        ImuController imu(imuI2c);

        RcCarDataManager dataManager;
        SeoilCoordController coordController;

        // [수정]
        // GPS가 얻은 lat/lon/pixel 값을 RcCarDataManager에 저장해야 하므로
        // 생성할 때 dataManager를 한 번 전달하고 내부에서 참조하도록 함.
        GpsController gps("/dev/serial0", dataManager);

        imu.initialize();

        servo.setCalibration(0, {-90.0, 90.0, 0.0, false, 500.0, 2500.0});
        servo.setCalibration(1, {-45.0, 45.0, 0.0, false, 500.0, 2500.0});
        servo.setCalibration(2, {-30.0, 30.0, 0.0, false, 500.0, 2500.0});

        servo.center(0);
        servo.center(1);
        servo.center(2);
        motor.stop();

        TerminalInput keyboard;
        Camera camera(640, 480, 30);

        // 여러 Thread가 읽고 쓰므로 atomic 사용.
        std::atomic<bool> running{true};

        // Main Thread가 command를 수정하고
        // Control Thread가 읽으므로 mutex로 보호.
        ControlCommand command;
        std::mutex commandMutex;

        // [수정]
        // 기존에는 Map Thread가 satelliteImg를 수정했지만,
        // 이제 GPS Thread가 satelliteImg를 수정함.
        // Main Thread는 계속 satelliteImg를 읽어서 화면에 출력하므로
        // mapMutex는 그대로 필요함.
        cv::Mat satelliteImg;
        std::mutex mapMutex;

        // Thread 객체를 먼저 빈 상태로 생성.
        //
        // [수정 이유]
        // Thread 생성 도중 예외가 발생해도
        // 이미 만들어진 Thread를 join할 수 있도록 하기 위함.
        std::thread controlThread;
        std::thread imuThread;
        std::thread gpsThread;

        // [제거]
        // std::thread mapThread;
        //
        // [제거 이유]
        // 지도 갱신을 GPS Thread에서 함께 처리하므로
        // 별도의 Map Thread가 필요하지 않음.

        // 전체 프로그램 종료 요청을 한 곳으로 통일.
        auto requestStop = [&]()
        {
            running.store(false);
            gps.stopThread();
        };

        // 생성된 모든 Thread를 안전하게 종료 대기.
        auto joinThreads = [&]()
        {
            if (controlThread.joinable())
                controlThread.join();

            if (imuThread.joinable())
                imuThread.join();

            if (gpsThread.joinable())
                gpsThread.join();

            // [제거]
            // if (mapThread.joinable())
            //     mapThread.join();
            //
            // [제거 이유]
            // Map Thread 자체가 없어졌으므로 join할 필요도 없음.
        };

        try
        {
            // ================= CONTROL THREAD =================
            controlThread = std::thread([&]()
                                        {
                // Hardware 초기값은 main에서 이미
                // motor.stop(), servo.center()로 맞춰놓은 상태.
                ControlCommand lastApplied;

                auto stopControlHardware = [&]()
                {
                    // 종료 중 Hardware 오류가 나더라도
                    // Thread 종료 자체는 계속 진행해야 하므로 각각 보호.
                    try { motor.stop(); } catch (...) {}
                    try { servo.center(0); } catch (...) {}
                    try { servo.center(1); } catch (...) {}
                    try { servo.center(2); } catch (...) {}
                };

                try
                {
                    while (running.load())
                    {
                        ControlCommand current;

                        {
                            std::lock_guard<std::mutex> lock(commandMutex);
                            current = command;
                        }

                        // 기존에는 10ms마다 같은 speed라도 motor.drive()를 계속 호출했음.
                        // MotorController::setSpeed() 내부에서는 PWM을 0으로 만든 뒤
                        // 10ms 대기하는 과정이 있으므로 같은 명령을 반복 호출하면
                        // 불필요하게 Motor 출력이 반복적으로 끊길 수 있음.
                        //
                        // 따라서 실제 명령이 바뀌었을 때만 Hardware에 적용.
                        if (current.speed != lastApplied.speed)
                        {
                            motor.drive(current.speed);
                            lastApplied.speed = current.speed;
                        }

                        if (current.steeringAngle != lastApplied.steeringAngle)
                        {
                            servo.setAngle(2, current.steeringAngle);
                            lastApplied.steeringAngle = current.steeringAngle;
                        }

                        if (current.cameraPan != lastApplied.cameraPan)
                        {
                            servo.setAngle(0, current.cameraPan);
                            lastApplied.cameraPan = current.cameraPan;
                        }

                        if (current.cameraTilt != lastApplied.cameraTilt)
                        {
                            servo.setAngle(1, current.cameraTilt);
                            lastApplied.cameraTilt = current.cameraTilt;
                        }

                        std::this_thread::sleep_for(
                            std::chrono::milliseconds(10)
                        );
                    }

                    stopControlHardware();
                }
                catch (const std::exception &e)
                {
                    std::cerr
                        << "[CONTROL THREAD ERROR] "
                        << e.what() << '\n';

                    stopControlHardware();
                    requestStop();
                }
                catch (...)
                {
                    std::cerr
                        << "[CONTROL THREAD ERROR] Unknown error\n";

                    stopControlHardware();
                    requestStop();
                } });

            // ================= IMU THREAD =================
            imuThread = std::thread([&]()
                                    {
                try
                {
                    auto lastPrint =
                        std::chrono::steady_clock::now();

                    while (running.load())
                    {
                        ImuData imuData = imu.read();

                        // IMU Thread → DataManager
                        dataManager.updateYaw(imuData.heading);

                        auto now =
                            std::chrono::steady_clock::now();

                        auto elapsed =
                            std::chrono::duration_cast<
                                std::chrono::milliseconds
                            >(now - lastPrint).count();

                        if (elapsed >= 500)
                        {
                            std::cout
                                << "[IMU] "
                                << "Heading=" << imuData.heading
                                << " Roll=" << imuData.roll
                                << " Pitch=" << imuData.pitch
                                << '\n';

                            lastPrint = now;
                        }

                        std::this_thread::sleep_for(
                            std::chrono::milliseconds(10)
                        );
                    }
                }
                catch (const std::exception &e)
                {
                    std::cerr
                        << "[IMU THREAD ERROR] "
                        << e.what() << '\n';

                    requestStop();
                }
                catch (...)
                {
                    std::cerr
                        << "[IMU THREAD ERROR] Unknown error\n";

                    requestStop();
                } });

            // ================= GPS THREAD =================
            gpsThread = std::thread([&]()
                                    {
                try
                {
                    // [수정]
                    // 기존:
                    // gps.runGpsThread(coordController);
                    //
                    // 이제 GPS 위치가 새로 저장될 때마다
                    // 뒤의 lambda를 실행해서 지도까지 함께 갱신함.
                    gps.runGpsThread(
                        coordController,
                        [&]()
                        {
                            // [Map Thread에서 이동]
                            // 새로운 GPS 데이터가 저장된 직후
                            // 최신 RC카 이동 경로를 가져옴.
                            auto path =
                                dataManager.getRcCarPath();

                            // 지도 그리기 역할 자체는 그대로
                            // SeoilCoordController가 담당함.
                            cv::Mat newMap =
                                coordController
                                    .drawPathOnSatelliteImg(path);

                            {
                                std::lock_guard<std::mutex> lock(mapMutex);

                                // GPS Thread가 새 지도 이미지를 저장하고
                                // Main Thread가 이 이미지를 화면에 출력함.
                                satelliteImg = newMap;
                            }
                        }
                    );
                }
                catch (const std::exception &e)
                {
                    std::cerr
                        << "[GPS THREAD ERROR] "
                        << e.what() << '\n';

                    requestStop();
                }
                catch (...)
                {
                    std::cerr
                        << "[GPS THREAD ERROR] Unknown error\n";

                    requestStop();
                } });

            std::cout
                << "==========================================\n"
                << " [조작 가이드]\n"
                << " W/S : 전진/후진\n"
                << " A/D : 좌/우 조향\n"
                << " I/K : 카메라 위/아래\n"
                << " J/L : 카메라 좌/우\n"
                << " R   : 카메라 리셋\n"
                << " Space : 정지\n"
                << " Q/ESC : 종료\n"
                << "==========================================\n";

            // ================= MAIN THREAD =================
            // Main Thread는 Camera + Keyboard + GUI만 담당.
            cv::Mat frame;
            constexpr double speedSetting = 30.0;

            while (running.load())
            {
                if (!camera.read(frame) || frame.empty())
                {
                    std::cerr
                        << "[ERROR] Failed to read camera frame.\n";

                    requestStop();
                    break;
                }

                cv::imshow("Robot Camera", frame);

                cv::Mat mapToShow;

                {
                    std::lock_guard<std::mutex> lock(mapMutex);

                    // GPS Thread가 만든 satelliteImg를
                    // Main Thread에서 안전하게 복사.
                    //
                    // imshow()는 mutex 밖에서 실행해서
                    // GPS Thread가 기다리는 시간을 줄임.
                    if (!satelliteImg.empty())
                        mapToShow = satelliteImg.clone();
                }

                if (!mapToShow.empty())
                    cv::imshow("Satellite Map", mapToShow);

                int cvKey = cv::waitKey(1);

                if (cvKey == 27)
                {
                    requestStop();
                    break;
                }

                int key = keyboard.readKey();

                if (key < 0)
                {
                    std::this_thread::sleep_for(
                        std::chrono::milliseconds(5));
                    continue;
                }

                {
                    std::lock_guard<std::mutex> lock(commandMutex);

                    switch (key)
                    {
                    case 'w':
                    case 'W':
                        command.speed = speedSetting;
                        break;

                    case 's':
                    case 'S':
                        command.speed = -speedSetting;
                        break;

                    case ' ':
                        command.speed = 0.0;
                        break;

                    case 'a':
                    case 'A':
                        command.steeringAngle -= 5.0;

                        if (command.steeringAngle < -30.0)
                            command.steeringAngle = -30.0;

                        break;

                    case 'd':
                    case 'D':
                        command.steeringAngle += 5.0;

                        if (command.steeringAngle > 30.0)
                            command.steeringAngle = 30.0;

                        break;

                    case 'i':
                    case 'I':
                        command.cameraTilt += 5.0;

                        if (command.cameraTilt > 45.0)
                            command.cameraTilt = 45.0;

                        break;

                    case 'k':
                    case 'K':
                        command.cameraTilt -= 5.0;

                        if (command.cameraTilt < -45.0)
                            command.cameraTilt = -45.0;

                        break;

                    case 'j':
                    case 'J':
                        command.cameraPan -= 5.0;

                        if (command.cameraPan < -90.0)
                            command.cameraPan = -90.0;

                        break;

                    case 'l':
                    case 'L':
                        command.cameraPan += 5.0;

                        if (command.cameraPan > 90.0)
                            command.cameraPan = 90.0;

                        break;

                    case 'r':
                    case 'R':
                        command.cameraPan = 0.0;
                        command.cameraTilt = 0.0;
                        break;

                    case 'q':
                    case 'Q':
                        requestStop();
                        break;
                    }
                }
            }

            // 정상 종료 시 모든 Thread에 종료 요청.
            requestStop();

            // Control / IMU / GPS Thread가 실제 종료될 때까지 대기.
            joinThreads();
        }
        catch (...)
        {
            // Thread 생성 이후 Main Thread에서 예외가 발생한 경우
            // join되지 않은 std::thread 객체가 소멸하면
            // std::terminate()가 발생할 수 있음.
            //
            // 따라서 반드시 종료 요청 → join 후
            // 원래 예외를 바깥 catch로 다시 전달.
            requestStop();
            joinThreads();
            throw;
        }

        cv::destroyAllWindows();
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << '\n';
        return 1;
    }
    catch (...)
    {
        std::cerr << "Unknown error\n";
        return 1;
    }

    return 0;
}