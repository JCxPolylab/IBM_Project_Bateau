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
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs videoio highgui dnn calib3d)

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
list(FILTER PROJECT_SOURCES EXCLUDE REGEX ".*example_.*_main\\.cpp$")

add_executable(CATJ_ibmRobotProject
    ${PROJECT_SOURCES}
)

target_include_directories(CATJ_ibmRobotProject PRIVATE
    "${CMAKE_SOURCE_DIR}"
    "${CMAKE_SOURCE_DIR}/sources"
    "${CMAKE_SOURCE_DIR}/sources/application"
)

# -----------------------------------------------------------------------------
# Optional: official SLAMTEC RPLIDAR SDK
# -----------------------------------------------------------------------------
if(USE_RPLIDAR_SDK)
    if(NOT EXISTS "${RPLIDAR_SDK_ROOT}/include/sl_lidar.h")
        message(FATAL_ERROR "RPLIDAR SDK not found: ${RPLIDAR_SDK_ROOT}")
    endif()

    # Common SDK sources
    file(GLOB_RECURSE RPLIDAR_SDK_COMMON_SOURCES CONFIGURE_DEPENDS
        "${RPLIDAR_SDK_ROOT}/src/*.cpp"
    )

    # Keep only platform-relevant arch sources.
    if(UNIX AND NOT APPLE)
        list(FILTER RPLIDAR_SDK_COMMON_SOURCES EXCLUDE REGEX ".*/arch/macOS/.*")
        list(FILTER RPLIDAR_SDK_COMMON_SOURCES EXCLUDE REGEX ".*/arch/win32/.*")
    elseif(APPLE)
        list(FILTER RPLIDAR_SDK_COMMON_SOURCES EXCLUDE REGEX ".*/arch/linux/.*")
        list(FILTER RPLIDAR_SDK_COMMON_SOURCES EXCLUDE REGEX ".*/arch/win32/.*")
    elseif(WIN32)
        list(FILTER RPLIDAR_SDK_COMMON_SOURCES EXCLUDE REGEX ".*/arch/linux/.*")
        list(FILTER RPLIDAR_SDK_COMMON_SOURCES EXCLUDE REGEX ".*/arch/macOS/.*")
    endif()

    add_library(catj_rplidar_sdk STATIC
        ${RPLIDAR_SDK_COMMON_SOURCES}
    )

    target_include_directories(catj_rplidar_sdk PUBLIC
        "${RPLIDAR_SDK_ROOT}/include"
        "${RPLIDAR_SDK_ROOT}/src"
        "${RPLIDAR_SDK_ROOT}/src/dataunpacker"
        "${RPLIDAR_SDK_ROOT}/src/dataunpacker/unpacker"
    )

    target_link_libraries(catj_rplidar_sdk PUBLIC Threads::Threads)

    if(WIN32)
        target_link_libraries(catj_rplidar_sdk PUBLIC ws2_32)
    endif()

    target_link_libraries(CATJ_ibmRobotProject PRIVATE catj_rplidar_sdk)
    target_compile_definitions(CATJ_ibmRobotProject PRIVATE CATJ_USE_RPLIDAR_SDK=1)

    message(STATUS "RPLIDAR SDK enabled from: ${RPLIDAR_SDK_ROOT}")
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
