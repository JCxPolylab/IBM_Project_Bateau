cmake_minimum_required(VERSION 3.16)
project(CATJ_ibmRobotProject LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- Sources ---
file(GLOB_RECURSE SRC_CPP CONFIGURE_DEPENDS
    "${CMAKE_SOURCE_DIR}/sources/*.cpp"
)

# Si tu as des fichiers "Copie" à exclure :
list(FILTER SRC_CPP EXCLUDE REGEX ".*Copie.*")

add_executable(CATJ_ibmRobotProject
    ${SRC_CPP}
)

# --- Includes ---
target_include_directories(CATJ_ibmRobotProject PRIVATE
    "${CMAKE_SOURCE_DIR}/sources"
    "${CMAKE_SOURCE_DIR}/sources/application"
)

# --- OpenCV ---
find_package(OpenCV REQUIRED)
target_link_libraries(CATJ_ibmRobotProject PRIVATE
    ${OpenCV_LIBS}
    pthread
)

# ---- ONNX Runtime (optionnel) ----
# Exemple: ORT_ROOT=/home/pi/libs/onnxruntime-linux-aarch64-1.xx.x
option(USE_ORT "Enable ONNX Runtime" OFF)
set(ORT_ROOT "${CMAKE_SOURCE_DIR}/onnxruntime" CACHE PATH "Path to ONNX Runtime root")

if(USE_ORT)
  if(EXISTS "${ORT_ROOT}/onnxruntime_cxx_api.h")
    message(STATUS "Using ONNX Runtime from: ${ORT_ROOT}")
    target_compile_definitions(CATJ_ibmRobotProject PRIVATE CATJ_USE_ORT=1)
    target_include_directories(CATJ_ibmRobotProject PRIVATE "${ORT_ROOT}/include")
    target_link_directories(CATJ_ibmRobotProject PRIVATE "${ORT_ROOT}/lib")
    target_link_libraries(CATJ_ibmRobotProject PRIVATE onnxruntime)
    set_target_properties(CATJ_ibmRobotProject PROPERTIES
      BUILD_RPATH "${ORT_ROOT}/lib"
      INSTALL_RPATH "${ORT_ROOT}/lib"
    )
  else()
    message(FATAL_ERROR "USE_ORT=ON but onnxruntime_cxx_api.h not found in ${ORT_ROOT}")
  endif()
endif()