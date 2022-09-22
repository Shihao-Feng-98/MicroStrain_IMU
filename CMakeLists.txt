cmake_minimum_required(VERSION 3.0.0)
project(IMU_test)
set(CMAKE_BUILD_TYPE "Debug")
# g++编译选项 -O3优化代码
set(CMAKE_CXX_FLAGS "$ENV{CXXFLAGS} -O3 -std=c++11 -Wall")

# 添加头文件路径 -I
include_directories(${CMAKE_SOURCE_DIR}/include
                    /usr/include/eigen3                 # Eigen库路径
                    /usr/share/c++-mscl/Boost/include/  # mscl库路径 
                    /usr/share/c++-mscl/source/         # mscl库路径
)

# 添加源文件路径
set(SRC 
    # src/microstrain_imu.cpp
) 

# 输出可执行文件 
add_executable(main src/main.cpp ${SRC})

# 链接库 -l
target_link_libraries(main PRIVATE 
                    /usr/share/c++-mscl/libmscl.so # IMU库
                    pthread           # C线程库 要在IMU库后面
)

# 可执行文件路径 bin
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
