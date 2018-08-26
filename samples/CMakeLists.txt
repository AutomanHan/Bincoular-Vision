cmake_minimum_required(VERSION 2.8)
project(mynteye_sdk_samples)

include(${PROJECT_SOURCE_DIR}/cmake/Common.cmake)

# flags

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")

#if(CMAKE_COMPILER_IS_GNUCC)
#  add_definitions(-D_GLIBCXX_USE_CXX11_ABI=0)
#endif()

include(${PROJECT_SOURCE_DIR}/cmake/DetectCXX11.cmake)

# variables

if(NOT SDK_DIR)
  get_filename_component(SDK_DIR ${PROJECT_SOURCE_DIR} DIRECTORY)
endif()
if(MINGW)
  set(SDK_LIB_DIR ${SDK_DIR}/bin)
else()
  set(SDK_LIB_DIR ${SDK_DIR}/lib)
endif()
if(OS_WIN AND NOT MINGW)
  set(LIB_MYNTEYE_CORE ${CMAKE_SHARED_LIBRARY_PREFIX}mynteye_core.lib)
else()
  set(LIB_MYNTEYE_CORE ${CMAKE_SHARED_LIBRARY_PREFIX}mynteye_core${CMAKE_SHARED_LIBRARY_SUFFIX})
endif()

message(STATUS "SDK_DIR: ${SDK_DIR}")
message(STATUS "SDK_LIB_DIR: ${SDK_LIB_DIR}")

# output

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

foreach(CONFIG ${CMAKE_CONFIGURATION_TYPES})
  string(TOUPPER ${CONFIG} CONFIG)
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${CONFIG} ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY})
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_${CONFIG} ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${CONFIG} ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
endforeach()

# required

include(${PROJECT_SOURCE_DIR}/cmake/DetectOpenCV.cmake)

include_directories(
  ${SDK_DIR}/include
)

# plugin

#add_definitions(-DVERBOSE)

add_library(plugin_ocv_bm SHARED
  src/plugin/plugin_ocv_bm.cc
)
target_link_libraries(plugin_ocv_bm
  ${SDK_LIB_DIR}/${LIB_MYNTEYE_CORE}
  ${OpenCV_LIBS}
)
target_compile_definitions(plugin_ocv_bm PRIVATE MYNTEYE_EXPORTS)

# camera

add_executable(camera src/camera.cc)
target_link_libraries(camera
  ${SDK_LIB_DIR}/${LIB_MYNTEYE_CORE}
  ${OpenCV_LIBS}
)

add_executable(camera2 src/camera2.cc)
target_link_libraries(camera2
  ${SDK_LIB_DIR}/${LIB_MYNTEYE_CORE}
  ${OpenCV_LIBS}
)

add_executable(camera3 src/camera3.cc)
target_link_libraries(camera3
  ${SDK_LIB_DIR}/${LIB_MYNTEYE_CORE}
  ${OpenCV_LIBS}
)

if(OS_WIN)
  set(SDK_SAMPLES_LIB_DIR ${PROJECT_SOURCE_DIR}/bin)
else()
  set(SDK_SAMPLES_LIB_DIR ${PROJECT_SOURCE_DIR}/lib)
endif()
add_definitions(-DSDK_SAMPLES_LIB_DIR=\"${SDK_SAMPLES_LIB_DIR}\")
add_definitions(-DLIBPLUGIN=\"${CMAKE_SHARED_LIBRARY_PREFIX}plugin_ocv_bm${CMAKE_SHARED_LIBRARY_SUFFIX}\")

add_executable(camera_with_plugin src/camera_with_plugin.cc)
target_link_libraries(camera_with_plugin
  ${SDK_LIB_DIR}/${LIB_MYNTEYE_CORE}
  ${OpenCV_LIBS}
)

add_executable(camera_kitti
  src/camera_kitti.cc
  src/compat/compat.cc
  src/dataset/dataset.cc
)
target_link_libraries(camera_kitti
  ${SDK_LIB_DIR}/${LIB_MYNTEYE_CORE}
  ${OpenCV_LIBS}
)

if(OS_LINUX)
  add_executable(camera_ctrl src/camera_ctrl.cc)
  target_link_libraries(camera_ctrl
    ${SDK_LIB_DIR}/${LIB_MYNTEYE_CORE}
    ${OpenCV_LIBS}
  )
endif()

add_executable(camera_test src/camera_test.cc)
target_link_libraries(camera_test
  ${SDK_LIB_DIR}/${LIB_MYNTEYE_CORE}
  ${OpenCV_LIBS}
)
