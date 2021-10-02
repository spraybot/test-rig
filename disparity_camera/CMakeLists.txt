cmake_minimum_required(VERSION 2.8.3)
project(disparity_camera)

# Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  cv_bridge
  image_transport
  message_filters
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

find_package(OpenCV 3.2 REQUIRED)
# find_package(CUDA ${ZED_CUDA_VERSION} EXACT REQUIRED)     # disable CUDA for CPU version

## System dependencies are found with CMake's conventions
#find_package(Boost REQUIRED COMPONENTS system)

# catkin_python_setup()

# Generate actions in the 'action' folder
 add_action_files(
   DIRECTORY action
   FILES disparityAction.action
 )

# Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   actionlib_msgs   sensor_msgs   std_msgs
 )

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS actionlib actionlib_msgs cv_bridge image_transport message_filters pcl_ros roscpp rospy sensor_msgs std_msgs
  DEPENDS system_lib
)

# #######################
# ## Build FOR GPU NODE##
# #######################

# include_directories( ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS})
# include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# set(
# 	CUDA_NVCC_FLAGS
# 	${CUDA_NVCC_FLAGS};
# 	-O3
# 	-gencode arch=compute_61,code=sm_61
# )
# set (CUDA_VERBOSE_BUILD ON CACHE BOOL "nvcc verbose" FORCE)
# #SET(CUDA_NVCC_FLAGS "-arch=sm_13" CACHE STRING "nvcc flags" FORCE)
# #SET (CUDA_VERBOSE_BUILD ON CACHE BOOL "nvcc verbose" FORCE)
# set(LIB_TYPE STATIC)
# CUDA_ADD_LIBRARY(bova_sgm ${LIB_TYPE} src/bova_sgm.cu)
# CUDA_ADD_LIBRARY(aggregation ${LIB_TYPE} src/aggregation.cu)
# CUDA_ADD_LIBRARY(costs ${LIB_TYPE} src/costs.cu)

# add_executable(${PROJECT_NAME}_server_gpu src/disparity_server_gpu.cpp)
# add_executable(${PROJECT_NAME}_client_gpu src/disparity_client_gpu.cpp)

# add_dependencies (${PROJECT_NAME}_server_gpu bova_sgm aggregation costs ${${PROJECT_NAME}_EXPORTED_TARGETS} ${CUDA_ADD_LIBRARY})
# add_dependencies (${PROJECT_NAME}_client_gpu  ${${PROJECT_NAME}_EXPORTED_TARGETS})

# target_link_libraries(${PROJECT_NAME}_server_gpu ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${CUDA_ADD_LIBRARY} bova_sgm aggregation costs ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY} ${CUDA_NPP_LIBRARIES_ZED} )
# target_link_libraries(${PROJECT_NAME}_client_gpu ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${CUDA_ADD_LIBRARY})


#######################
## Build FOR CPU NODE##
# Comment GPU NODE above to compile
#######################

include_directories( ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(${PROJECT_NAME}_server_cpu src/disparity_server_cpu.cpp)
add_executable(${PROJECT_NAME}_client_cpu src/disparity_client_cpu.cpp)

add_dependencies (${PROJECT_NAME}_server_cpu  ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies (${PROJECT_NAME}_client_cpu  ${${PROJECT_NAME}_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_server_cpu ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_client_cpu ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

