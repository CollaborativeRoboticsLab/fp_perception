cmake_minimum_required(VERSION 3.14)
project(pytorch_vendor)

set(FETCHCONTENT_QUIET OFF)
include(FetchContent)
find_package(ament_cmake REQUIRED)

set(PYTORCH_VERSION "2.7.0" CACHE STRING "Version of LibTorch to download") # User-defined Torch version (default: 2.7.0)
set(LIBTORCH_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/libtorch)             # Use a persistent path in the source tree

# Skip download if already exists
if(EXISTS "${LIBTORCH_DIR}/lib/libtorch.so")
  message(STATUS "📦 Found existing LibTorch at ${LIBTORCH_DIR}, skipping download.")
else()
  message(STATUS "⬇️  Downloading LibTorch ${PYTORCH_VERSION}...")

  set(USE_CUDA FALSE)

  # Try to detect CUDA
  find_program(NVCC_PATH nvcc)
  if(NVCC_PATH)
    execute_process(COMMAND nvcc --version OUTPUT_VARIABLE CUDA_OUTPUT OUTPUT_STRIP_TRAILING_WHITESPACE)
    string(REGEX MATCH "release ([0-9]+)\\.([0-9]+)" _ ${CUDA_OUTPUT})
    set(CUDA_VERSION "${CMAKE_MATCH_1}${CMAKE_MATCH_2}")
    message(STATUS "🧠 Detected CUDA version: ${CUDA_VERSION}")
    set(USE_CUDA TRUE)
  else()
    message(WARNING "⚠️  CUDA not found. Falling back to CPU-only LibTorch.")
  endif()

  # Determine download URL
  if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
    if(USE_CUDA)
      set(LIBTORCH_URL "https://download.pytorch.org/libtorch/cu${CUDA_VERSION}/libtorch-cxx11-abi-shared-with-deps-${PYTORCH_VERSION}%2Bcu${CUDA_VERSION}.zip")
    else()
      set(LIBTORCH_URL "https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-${PYTORCH_VERSION}%2Bcpu.zip")
    endif()
  elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
    if(USE_CUDA)
      message(WARNING "⚠️ CUDA builds for aarch64 are not officially available. Falling back to CPU.")
    endif()
    set(LIBTORCH_URL "https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-${PYTORCH_VERSION}%2Bcpu.zip")
  else()
    message(FATAL_ERROR "❌ Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
  endif()

  message(STATUS "📥 Downloading LibTorch from: ${LIBTORCH_URL}")

  # Download and extract
  FetchContent_Declare(
    libtorch_vendor
    URL ${LIBTORCH_URL}
  )
  FetchContent_Populate(libtorch_vendor)

    
  file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/external)   # Ensure external directory exists
  file(RENAME ${libtorch_vendor_SOURCE_DIR} ${LIBTORCH_DIR})  # Move downloaded content to persistent external/libtorch
  message(STATUS "✅ LibTorch extracted to: ${LIBTORCH_DIR}")
endif()

# Copy headers/libs to install space during colcon build
install(DIRECTORY ${LIBTORCH_DIR}/include/ DESTINATION include)
install(DIRECTORY ${LIBTORCH_DIR}/lib/ DESTINATION lib)

# Create a local-only interface target (no install)
add_library(pytorch_vendor INTERFACE)

target_include_directories(pytorch_vendor INTERFACE
  $<BUILD_INTERFACE:${LIBTORCH_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_directories(pytorch_vendor INTERFACE
  $<BUILD_INTERFACE:${LIBTORCH_DIR}/lib>
  $<INSTALL_INTERFACE:lib>
)

target_link_libraries(pytorch_vendor INTERFACE torch c10)

install(TARGETS pytorch_vendor
  EXPORT pytorch_vendor   # This line is REQUIRED before install(EXPORT...)
)

install(EXPORT pytorch_vendor
  FILE pytorch_vendorTargets.cmake
  DESTINATION share/pytorch_vendor/cmake
)

ament_export_targets(pytorch_vendor)
ament_export_include_directories(include)
ament_export_libraries(torch c10)
ament_package()