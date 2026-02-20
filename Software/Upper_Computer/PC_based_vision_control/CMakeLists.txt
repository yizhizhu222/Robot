cmake_minimum_required(VERSION 3.0)
project(yolo_robot)

# --- 1. 寻找 OpenCV ---
find_package(OpenCV REQUIRED)

# --- 2. 寻找 ncnn ---
find_package(ncnn REQUIRED)

# --- 3. 开启 OpenMP ---
find_package(OpenMP REQUIRED)

# --- 4. 头文件与包含 ---
include_directories(${OpenCV_INCLUDE_DIRS})

# --- 5. 生成可执行文件 ---
add_executable(robot_eye main.cpp)

# --- 6. 链接库文件 ---
target_link_libraries(robot_eye ncnn ${OpenCV_LIBS} OpenMP::OpenMP_CXX)