cmake_minimum_required(VERSION 3.10)

project(cyberdog_motor_sdk VERSION 1.0.0)  # 添加版本号，便于find_package

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-Wno-shadow -Wno-unused-parameter)

# Find required packages
find_package(Threads REQUIRED)

# Explicitly set serial_DIR and find serial package
set(serial_DIR "/usr/local/share/serial/cmake")
find_package(serial REQUIRED)

# Include directories（这些是构建时用的，安装后会导出）
include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/include/socketcan
    ${PROJECT_SOURCE_DIR}/include/yesense
)

# Source files（排除main.cpp，因为库不需要main；如果需要测试可执行文件，可以单独添加）
set(SOURCES
    ${PROJECT_SOURCE_DIR}/src/CustomInterface.cpp
    ${PROJECT_SOURCE_DIR}/src/CyberdogInterface.cpp
    ${PROJECT_SOURCE_DIR}/src/socketcan/math_ops.cpp
    ${PROJECT_SOURCE_DIR}/src/socketcan/se_motor.cpp
    ${PROJECT_SOURCE_DIR}/src/socketcan/socketcan_driver.cpp
    ${PROJECT_SOURCE_DIR}/src/yesense/analysis_data.c
    ${PROJECT_SOURCE_DIR}/src/yesense/yesense_driver.cpp
)

# Add library（改为add_library，共享库）
add_library(${PROJECT_NAME} SHARED ${SOURCES})  # 或STATIC，如果不需要共享

# Link libraries（公开链接依赖，便于其他包继承）
target_link_libraries(${PROJECT_NAME}
    PUBLIC Threads::Threads  # PUBLIC让依赖传递
    PRIVATE serial  # PRIVATE只内部用
)

# 设置include目录为PUBLIC，便于其他包使用
target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>  # 安装后的路径
    PRIVATE
        ${PROJECT_SOURCE_DIR}/include/socketcan
        ${PROJECT_SOURCE_DIR}/include/yesense
)

# Install library and headers
install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# Install all header files（递归安装include目录下所有文件）
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/
    DESTINATION include/${PROJECT_NAME}
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

# Export CMake config for find_package
install(EXPORT ${PROJECT_NAME}Targets
    FILE ${PROJECT_NAME}Targets.cmake
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION lib/cmake/${PROJECT_NAME}
)

# Generate config file
include(CMakePackageConfigHelpers)
configure_package_config_file(
    ${PROJECT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in
    ${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
)

write_basic_package_version_file(
    ${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

install(FILES
    ${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    ${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
    DESTINATION lib/cmake/${PROJECT_NAME}
)

# 如果需要一个测试可执行文件，可以添加（可选）
add_executable(${PROJECT_NAME}_test ${PROJECT_SOURCE_DIR}/src/main.cpp)
target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})