cmake_minimum_required(VERSION 3.14)

project(h2au_project LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)

add_executable(h2au_app
     main.cpp
    I2cDevice.cpp
    PwmController.cpp
    ServoController.cpp
    MotorController.cpp
    GpioOutput.cpp
    TerminalInput.cpp
    Camera.cpp
    ImuController.cpp
    GpsController.cpp
    SeoilCoordController.cpp
    RcCarDataManager.cpp
)

target_include_directories(h2au_app PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(h2au_app PRIVATE
    ${OpenCV_LIBS}
)

if(MSVC)
    target_compile_options(h2au_app PRIVATE /EHsc)
endif()