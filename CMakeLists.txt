cmake_minimum_required(VERSION 4.0)
project(LeapTest)

set(CMAKE_CXX_STANDARD 20)

# Adjust this path if your SDK is installed elsewhere
set(LEAP_SDK_DIR "C:/Program Files/Ultraleap/LeapSDK")

include_directories("${LEAP_SDK_DIR}/include")
link_directories("${LEAP_SDK_DIR}/lib/x64")

add_executable(LeapTest main.cpp)
target_link_libraries(LeapTest LeapC)

add_executable(LeapDroneGui
        main.cpp

        third_party/imgui/imgui.cpp
        third_party/imgui/imgui_draw.cpp
        third_party/imgui/imgui_tables.cpp
        third_party/imgui/imgui_widgets.cpp
        third_party/imgui/imgui_demo.cpp

        third_party/imgui/backends/imgui_impl_win32.cpp
        third_party/imgui/backends/imgui_impl_dx11.cpp
)

target_include_directories(LeapDroneGui PRIVATE
        third_party/imgui
        third_party/imgui/backends
)

target_compile_definitions(LeapDroneGui PRIVATE
        WIN32_LEAN_AND_MEAN
        NOMINMAX
)

target_link_libraries(LeapDroneGui PRIVATE
        ws2_32
        d3d11
        dxgi
        d3dcompiler
)

if (MSVC)
    target_compile_options(LeapDroneGui PRIVATE /W3)
endif()