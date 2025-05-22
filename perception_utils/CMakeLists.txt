cmake_minimum_required(VERSION 3.8)
project(perception_utils)

include(FetchContent)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)

# Determine CPU and set TensorFlow Lite URL
execute_process(
  COMMAND uname -m
  OUTPUT_VARIABLE CPU_ARCH
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

set(TFLITE_URL "")

if(CPU_ARCH STREQUAL "aarch64")
  message(STATUS "Detected ARM64 architecture")
  set(TFLITE_URL "https://github.com/CLFML/TensorFlow_Lite_Compiled/releases/download/v2.16.1/tensorflow_linux_generic_aarch64.zip")
elseif(CPU_ARCH STREQUAL "x86_64")
  # Check for AVX2 support
  execute_process(
    COMMAND lscpu
    OUTPUT_VARIABLE CPU_INFO
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  if(CPU_INFO MATCHES "avx2")
    message(STATUS "Detected x86_64 with AVX2 support")
    set(TFLITE_URL "https://github.com/CLFML/TensorFlow_Lite_Compiled/releases/download/v2.16.1/tensorflow_linux_avx2_x86_64.zip")
  elseif(CPU_INFO MATCHES "avx")
    message(STATUS "Detected x86_64 with AVX support")
    set(TFLITE_URL "https://github.com/CLFML/TensorFlow_Lite_Compiled/releases/download/v2.16.1/tensorflow_linux_generic_x86_64.zip")
  else()
    message(FATAL_ERROR "x86_64 detected but no AVX or AVX2 support found. Please use a supported CPU or build TensorFlow Lite from source.")
  endif()
else()
  message(FATAL_ERROR "Unsupported architecture: ${CPU_ARCH}")
endif()

# Fetch TensorFlow Lite precompiled
FetchContent_Declare(
  tflite_precompiled
  URL ${TFLITE_URL}
  SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/external/tensorflow
  SOURCE_SUBDIR tensorflow/lite
)

FetchContent_MakeAvailable(tflite_precompiled)

# Include paths
include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}/external/tensorflow/inc
)

link_directories(${CMAKE_CURRENT_BINARY_DIR}/external/tensorflow/lib)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install TensorFlow headers and libs
install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/external/tensorflow/inc/
  DESTINATION include
)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/external/tensorflow/lib/libtensorflowlite.so
  DESTINATION lib
)

# Export include directory for use in dependent packages
ament_export_include_directories(include)
ament_package()
