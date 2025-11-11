include_directories(${CMAKE_BINARY_DIR})
include_directories(${PROJECT_SOURCE_DIR})
include_directories("include/")
include_directories("./")
include_directories("../common/include/")
include_directories("../third-party")
include_directories("../third-party/ParamHandler")
include_directories("../third-party/vectornav/include")
include_directories("../third-party/vectornav/include/vn")
include_directories("../third-party/lord_imu/Include")

include_directories("../lcm-types/cpp")
include_directories("/usr/local/include/lcm/")   # lcm includes
include_directories("../third-party/SOEM/osal")
include_directories("../third-party/SOEM/osal/linux")
include_directories("../third-party/SOEM/oshw")
include_directories("../third-party/SOEM/oshw/linux")

file(GLOB sources "src/*.cpp" "src/rt/*.c"  "src/rt/*.cpp")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-error=stringop-truncation")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-error=stringop-truncation")


add_library(robot SHARED ${sources})

target_link_libraries(robot biomimetics pthread lcm inih dynacore_param_handler lord_imu soem)
if(CMAKE_SYSTEM_NAME MATCHES Linux)
target_link_libraries(robot libvnc rt)
endif()
