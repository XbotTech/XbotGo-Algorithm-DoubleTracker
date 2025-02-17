cmake_minimum_required(VERSION 3.14)

project(XbotgoSDK)

set(CMAKE_CXX_STANDARD 17)

option(MAKE_UNIT_TEST "编译单元测试" OFF)
enable_testing()

if(DEFINED ANDROID)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
    set(CMAKE_POSITION_INDEPENDENT_CODE ON)
endif()

if(DEFINED MACOS)
    set(CMAKE_OSX_SYSROOT "/Library/Developer/CommandLineTools/SDKs/MacOSX13.3.sdk")
endif()

if(${CMAKE_BUILD_TYPE} MATCHES "Debug")
    message("Build Type: Debug")
    set(CMAKE_CXX_FLAGS_DEBUG "-Wall -g")
else()
    message("Build Type: Release")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3")
endif()

# 编译选项定义
add_definitions(-DGLOG_USE_GLOG_EXPORT)

# 设置输出路径
set(LIBRARY_OUTPUT_PATH "${CMAKE_BINARY_DIR}/lib")

# 包含所有相关的头文件目录
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/sdk/
    ${CMAKE_CURRENT_SOURCE_DIR}/sdk/core/
    ${CMAKE_CURRENT_SOURCE_DIR}/sdk/logic/module/
    ${CMAKE_CURRENT_SOURCE_DIR}/sdk/logic/module/bytetracker/
    ${CMAKE_CURRENT_SOURCE_DIR}/sdk/logic/module/bytetracker/ByteTrack/
    ${CMAKE_CURRENT_SOURCE_DIR}/sdk/logic/module/bytetracker/ByteTrack/Eigen
    ${CMAKE_CURRENT_SOURCE_DIR}/sdk/logic/strategy/
    ${CMAKE_CURRENT_SOURCE_DIR}/sdk/logic/strategy/gimbal/
    ${CMAKE_CURRENT_SOURCE_DIR}/sdk/logic/strategy/chameleon/
    ${CMAKE_CURRENT_SOURCE_DIR}/sdk/logic/util/
    ${CMAKE_CURRENT_SOURCE_DIR}/wrapper
    ${CMAKE_CURRENT_SOURCE_DIR}/wrapper/api
    ${CMAKE_CURRENT_SOURCE_DIR}/wrapper/callback
    ${CMAKE_CURRENT_SOURCE_DIR}/external/json
)

# 获取所有源文件
file(GLOB_RECURSE core_sdk_src_files "${CMAKE_SOURCE_DIR}/sdk/core/*.cpp" "${CMAKE_SOURCE_DIR}/sdk/core/*.h")
file(GLOB_RECURSE logic_sdk_src_files "${CMAKE_SOURCE_DIR}/sdk/logic/*.cpp" "${CMAKE_SOURCE_DIR}/sdk/logic/*.h")
file(GLOB_RECURSE wrapper_src_files "${CMAKE_SOURCE_DIR}/wrapper/*.cpp" "${CMAKE_SOURCE_DIR}/wrapper/*.h")

# 将所有源文件合并到一起
set(all_src_files ${core_sdk_src_files} ${logic_sdk_src_files} ${wrapper_src_files})

# 定义共享库
add_library(xbotgo_sdk SHARED ${all_src_files})

# 添加可执行文件
set(src main.cpp)
add_executable(main ${src})
target_link_libraries(main xbotgo_sdk)
set_target_properties(main PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)
set_target_properties(xbotgo_sdk PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)