cmake_minimum_required(VERSION 3.16)
project(CATJ_ibmRobotProject LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# -----------------------------------------------------------------------------
# Options
# -----------------------------------------------------------------------------
option(USE_ORT "Enable ONNX Runtime backend" OFF)
option(USE_RPLIDAR_SDK "Enable official SLAMTEC RPLIDAR SDK" ON)

set(ORT_ROOT "${CMAKE_SOURCE_DIR}/onnxruntime" CACHE PATH "Path to ONNX Runtime root folder")
set(RPLIDAR_SDK_ROOT "${CMAKE_SOURCE_DIR}/third_party/rplidar_sdk/sdk" CACHE PATH "Path to SLAMTEC RPLIDAR SDK sdk folder")

# -----------------------------------------------------------------------------
# Dependencies
# -----------------------------------------------------------------------------
find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs videoio highgui dnn)

# -----------------------------------------------------------------------------
# Project sources
# -----------------------------------------------------------------------------
file(GLOB_RECURSE PROJECT_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_SOURCE_DIR}/sources/*.cpp"
)

# Exclude archived / example sources if they ever appear as .cpp again.
list(FILTER PROJECT_SOURCES EXCLUDE REGEX ".*/example/.*")
list(FILTER PROJECT_SOURCES EXCLUDE REGEX ".*OLD_main\\.cpp$")
list(FILTER PROJECT_SOURCES EXCLUDE REGEX ".*main - Copie\\.cpp$")

add_executable(CATJ_ibmRobotProject
    ${PROJECT_SOURCES}
)

# Base includes for your project code.
target_include_directories(CATJ_ibmRobotProject PRIVATE
    "${CMAKE_SOURCE_DIR}"
    "${CMAKE_SOURCE_DIR}/sources"
    "${CMAKE_SOURCE_DIR}/sources/application"
)

# -----------------------------------------------------------------------------
# Optional: official SLAMTEC RPLIDAR SDK
# Your current code in sources/fonctions/capteur/lidar/Lidar.cpp uses:
#   #include "sl_lidar.h"
#   #include "sl_lidar_driver.h"
# so the SDK include path must expose third_party/rplidar_sdk/sdk/include.
# -----------------------------------------------------------------------------
if(USE_RPLIDAR_SDK)
    set(RPLIDAR_REQUIRED_FILES
        "${RPLIDAR_SDK_ROOT}/include/sl_lidar.h"
        "${RPLIDAR_SDK_ROOT}/include/sl_lidar_driver.h"
        "${RPLIDAR_SDK_ROOT}/src/sl_lidar_driver.cpp"
        "${RPLIDAR_SDK_ROOT}/src/sl_serial_channel.cpp"
        "${RPLIDAR_SDK_ROOT}/src/hal/thread.cpp"
        "${RPLIDAR_SDK_ROOT}/src/dataunpacker/dataunpacker.cpp"
    )

    set(RPLIDAR_SDK_OK TRUE)
    foreach(_f IN LISTS RPLIDAR_REQUIRED_FILES)
        if(NOT EXISTS "${_f}")
            set(RPLIDAR_SDK_OK FALSE)
            message(WARNING "RPLIDAR SDK file missing: ${_f}")
        endif()
    endforeach()

    if(RPLIDAR_SDK_OK)
        add_library(catj_rplidar_sdk STATIC
            "${RPLIDAR_SDK_ROOT}/src/hal/thread.cpp"
            "${RPLIDAR_SDK_ROOT}/src/dataunpacker/dataunpacker.cpp"
            "${RPLIDAR_SDK_ROOT}/src/sl_async_transceiver.cpp"
            "${RPLIDAR_SDK_ROOT}/src/sl_crc.cpp"
            "${RPLIDAR_SDK_ROOT}/src/sl_lidarprotocol_codec.cpp"
            "${RPLIDAR_SDK_ROOT}/src/sl_lidar_driver.cpp"
            "${RPLIDAR_SDK_ROOT}/src/sl_serial_channel.cpp"
            "${RPLIDAR_SDK_ROOT}/src/sl_tcp_channel.cpp"
            "${RPLIDAR_SDK_ROOT}/src/sl_udp_channel.cpp"
        )

        target_include_directories(catj_rplidar_sdk PUBLIC
            "${RPLIDAR_SDK_ROOT}/include"
            "${RPLIDAR_SDK_ROOT}/src"
        )

        target_link_libraries(catj_rplidar_sdk PUBLIC Threads::Threads)
        target_link_libraries(CATJ_ibmRobotProject PRIVATE catj_rplidar_sdk)
        target_compile_definitions(CATJ_ibmRobotProject PRIVATE CATJ_USE_RPLIDAR_SDK=1)

        message(STATUS "RPLIDAR SDK enabled from: ${RPLIDAR_SDK_ROOT}")
    else()
        message(WARNING "RPLIDAR SDK requested but incomplete. Build will continue; your Lidar.cpp will fall back to mock mode.")
    endif()
endif()

# -----------------------------------------------------------------------------
# Optional: ONNX Runtime
# -----------------------------------------------------------------------------
if(USE_ORT)
    if(EXISTS "${ORT_ROOT}/onnxruntime_cxx_api.h")
        target_compile_definitions(CATJ_ibmRobotProject PRIVATE CATJ_USE_ORT=1)
        target_include_directories(CATJ_ibmRobotProject PRIVATE "${ORT_ROOT}")

        find_library(ONNXRUNTIME_LIB
            NAMES onnxruntime
            HINTS "${ORT_ROOT}/lib"
            NO_DEFAULT_PATH
        )

        if(NOT ONNXRUNTIME_LIB)
            message(FATAL_ERROR "USE_ORT=ON but libonnxruntime not found in ${ORT_ROOT}/lib")
        endif()

        target_link_libraries(CATJ_ibmRobotProject PRIVATE "${ONNXRUNTIME_LIB}")
        set_target_properties(CATJ_ibmRobotProject PROPERTIES
            BUILD_RPATH "${ORT_ROOT}/lib"
            INSTALL_RPATH "${ORT_ROOT}/lib"
        )

        message(STATUS "ONNX Runtime enabled from: ${ORT_ROOT}")
    else()
        message(FATAL_ERROR "USE_ORT=ON but onnxruntime_cxx_api.h not found in ${ORT_ROOT}")
    endif()
endif()

# -----------------------------------------------------------------------------
# Final link
# -----------------------------------------------------------------------------
target_link_libraries(CATJ_ibmRobotProject PRIVATE
    ${OpenCV_LIBS}
    Threads::Threads
)

# BlueZ is needed only if bluetooth headers are present and bluetooth.cpp really
# compiles the Linux RFCOMM implementation. Linking it when found is harmless.
if(UNIX AND NOT APPLE)
    find_library(BLUETOOTH_LIB bluetooth)
    if(BLUETOOTH_LIB)
        target_link_libraries(CATJ_ibmRobotProject PRIVATE ${BLUETOOTH_LIB})
        message(STATUS "BlueZ library found: ${BLUETOOTH_LIB}")
    else()
        message(STATUS "BlueZ library not found: bluetooth.cpp will still compile in stub mode if headers are absent.")
    endif()
endif()

message(STATUS "OpenCV version: ${OpenCV_VERSION}")
message(STATUS "Project source count: ${PROJECT_SOURCES}")
