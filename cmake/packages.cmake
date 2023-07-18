# 引入该目录下的.cmake文件
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

include_directories(${CMAKE_SOURCE_DIR}/../devel/include) # 引用ros生成的msg header


#       system config
message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" )
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processer number:  ${N}")
  if(N GREATER 4)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=3)
    message("core for MP: 3")
  elseif(N GREATER 3)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=2)
    message("core for MP: 2")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

# eigen 3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

# sophus
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/sophus)

# glog
find_package(Glog REQUIRED)
include_directories(${Glog_INCLUDE_DIRS})

# csparse
find_package(CSparse REQUIRED)
include_directories(${CSPARSE_INCLUDE_DIR})

# cholmod
find_package(Cholmod REQUIRED)
include_directories(${CHOLMOD_INCLUDE_DIRS})

# pcl
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

# opencv
find_package(OpenCV REQUIRED)
# find_package(OpenCV 4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_libraries(${CERES_LIBRARY_DIRS})

#ceres
find_package(Ceres 2 REQUIRED PATHS /home/zhaochengwei/workspace/3rdparty/ceres_210)
# find_package(Ceres REQUIRED PATHS /home/zhaochengwei/workspace/3rdparty/ceres_1.14)
include_directories( ${CERES_INCLUDE_DIRS})
link_directories(${CERES_LIBRARY_DIRS})


set(3RDPARTY_DIR ${PROJECT_SOURCE_DIR}/../../../workspace/3rdparty)


# find_package(G2O REQUIRED PATHS /home/zhaochengwei/workspace/3rdparty/g2o-20201223)
set(G2O_INCLUDE_DIRS ${3RDPARTY_DIR}/g2o-20201223/include)
set(G2O_LIBRARY_DIRS ${3RDPARTY_DIR}/g2o-20201223/lib)
# file(GLOB G2O_LIBRARIES ${G2O_LIBRARY_DIRS}/*.a ${G2O_LIBRARY_DIRS}/*.so*)

# g2o 使用thirdparty中的
# include_directories(${PROJECT_SOURCE_DIR}/thirdparty/g2o/)
include_directories(${G2O_INCLUDE_DIRS})
set(g2o_libs
        ${G2O_LIBRARY_DIRS}/libg2o_stuff.so
        ${G2O_LIBRARY_DIRS}/libg2o_core.so
	# ${G2O_LIBRARY_DIRS}/libg2o_solver_cholmod.so
        ${G2O_LIBRARY_DIRS}/libg2o_solver_dense.so
        ${G2O_LIBRARY_DIRS}/libg2o_solver_csparse.so
        ${G2O_LIBRARY_DIRS}/libg2o_csparse_extension.so
        ${G2O_LIBRARY_DIRS}/libg2o_types_sba.so
        ${CSPARSE_LIBRARY}
        ${CHOLMOD_LIBRARY}
        )

# ros
# 为了2D scan, pointcloud2
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        sensor_msgs
        pcl_ros
        pcl_conversions
        )
include_directories(${catkin_INCLUDE_DIRS})

# yaml-cpp
find_package(yaml-cpp REQUIRED)
include_directories(${yaml-cpp_INCLUDE_DIRS})

# 其他thirdparty下的内容
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/)

## third part
include(FetchContent)

FetchContent_Declare(
        tessil 
	SOURCE_DIR ${PROJECT_SOURCE_DIR}/thirdparty/tessil-src)

if (NOT tessil_POPULATED)
    set(BUILD_TESTING OFF)
    FetchContent_Populate(tessil)

    add_library(robin_map INTERFACE)
    add_library(tsl::robin_map ALIAS robin_map)

    target_include_directories(robin_map INTERFACE
            "$<BUILD_INTERFACE:${tessil_SOURCE_DIR}/include>")

    list(APPEND headers "${tessil_SOURCE_DIR}/include/tsl/robin_growth_policy.h"
            "${tessil_SOURCE_DIR}/include/tsl/robin_hash.h"
            "${tessil_SOURCE_DIR}/include/tsl/robin_map.h"
            "${tessil_SOURCE_DIR}/include/tsl/robin_set.h")
    target_sources(robin_map INTERFACE "$<BUILD_INTERFACE:${headers}>")

    if (MSVC)
        target_sources(robin_map INTERFACE
                "$<BUILD_INTERFACE:${tessil_SOURCE_DIR}/tsl-robin-map.natvis>")
    endif ()
endif ()



    set(third_party_libs
            ${catkin_LIBRARIES}
            ${g2o_libs}
            ${OpenCV_LIBS}
            ${PCL_LIBRARIES}
            ${CERES_LIBRARIES}
        #     ${Pangolin_LIBRARIES}
            glog gflags
            ${yaml-cpp_LIBRARIES}
            yaml-cpp
        #     TBB::tbb
        robin_map
            )
