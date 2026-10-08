cmake_minimum_required(VERSION 3.10)
project(rk3576_drv_daemon CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -Wall -Wextra -Wno-unused-parameter")

include_directories(include)

# ---------- 可选：真实视觉管线依赖（板端开启） ----------
# cmake .. -DUSE_RKNN=ON -DUSE_RGA=ON
option(USE_RKNN "Enable RKNN NPU inference" OFF)
option(USE_RGA  "Enable librga hardware CSC/resize" OFF)

add_executable(rk3576_app
    src/main.cc
    src/crc.cc
    src/workers.cc
    src/fusion.cc
    src/vision.cc
)

if(USE_RKNN)
    add_definitions(-DHAVE_RKNN)
    find_library(RKNN_LIB rknnrt PATHS /usr/lib /usr/local/lib)
    if(RKNN_LIB)
        target_link_libraries(rk3576_app ${RKNN_LIB})
    else()
        message(WARNING "rknnrt not found; comment out USE_RKNN")
    endif()
endif()

if(USE_RGA)
    add_definitions(-DHAVE_RGA)
    find_library(RGA_LIB rga PATHS /usr/lib /usr/local/lib)
    if(RGA_LIB)
        target_link_libraries(rk3576_app ${RGA_LIB})
    else()
        message(WARNING "librga not found; comment out USE_RGA")
    endif()
endif()

target_link_libraries(rk3576_app pthread)
