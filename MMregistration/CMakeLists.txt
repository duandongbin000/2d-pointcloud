cmake_minimum_required(VERSION 3.0)    #cmake最小版本
project(ManualMultRegistration)    #  工程名字
include_directories(include) #头文件路径
aux_source_directory(src DIR_SRCS)
set(TEST_MATH ${DIR_SRCS})
find_package(PCL 1.8 REQUIRED)  #寻找pcl包
include_directories(${PCL_INCLUDE_DIRS})   #头文件路径
link_directories(${PCL_LIBRARY_DIRS})  #连接库路径
add_definitions(${PCL_DEFINITIONS}) #添加编译参数
add_executable(ManualMultRegistration ${TEST_MATH})  #生成可执行文件
target_link_libraries (ManualMultRegistration ${PCL_LIBRARIES}) #添加需要的库文件名
#install(TARGETS ManualMultRegistration RUNTIME DESTINATION bin) #安装

