cmake_minimum_required(VERSION 4.0)
project(LeapTest)

set(CMAKE_CXX_STANDARD 20)

# Adjust this path if your SDK is installed elsewhere
set(LEAP_SDK_DIR "C:/Program Files/Ultraleap/LeapSDK")

include_directories("${LEAP_SDK_DIR}/include")
link_directories("${LEAP_SDK_DIR}/lib/x64")

add_executable(LeapTest main.cpp)
target_link_libraries(LeapTest LeapC)