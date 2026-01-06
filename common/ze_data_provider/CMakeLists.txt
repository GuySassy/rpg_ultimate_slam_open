project(ze_data_provider)
cmake_minimum_required(VERSION 2.8.3)

find_package(catkin_simple REQUIRED)
catkin_simple()

include(ze_setup)

#############
# LIBRARIES #
#############
set(HEADERS
  include/ze/data_provider/data_provider_base.hpp
  include/ze/data_provider/data_provider_factory.hpp
  include/ze/data_provider/data_provider_csv.hpp
  include/ze/data_provider/data_provider_rosbag.hpp
  include/ze/data_provider/data_provider_rostopic.hpp
  include/ze/data_provider/data_provider_rt1060.hpp
  include/ze/data_provider/camera_imu_synchronizer_base.hpp
  include/ze/data_provider/camera_imu_synchronizer.hpp
  )

set(SOURCES
  src/data_provider_base.cpp
  src/data_provider_factory.cpp
  src/data_provider_csv.cpp
  src/data_provider_rosbag.cpp
  src/data_provider_rostopic.cpp
  src/data_provider_rt1060.cpp
  src/camera_imu_synchronizer.cpp
  src/camera_imu_synchronizer_base.cpp
  )

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})

set(ELIS_LINK_ROOT_DIR "${ELIS_LINK_ROOT_DIR}" CACHE PATH "Path to elisUltimateLink root")
set(ELIS_LINK_INCLUDE_HINTS
  "${PROJECT_SOURCE_DIR}/../elisUltimateLink/include")
if(ELIS_LINK_ROOT_DIR)
  list(APPEND ELIS_LINK_INCLUDE_HINTS "${ELIS_LINK_ROOT_DIR}/include")
endif()
find_path(ELIS_LINK_INCLUDE_DIR elisUltimateLink/keypoint_service.hpp
  PATHS ${ELIS_LINK_INCLUDE_HINTS}
  NO_DEFAULT_PATH)
if(ELIS_LINK_INCLUDE_DIR)
  include_directories(${ELIS_LINK_INCLUDE_DIR})
endif()

set(ELIS_LINK_BUILD_DIR "${PROJECT_SOURCE_DIR}/../elisUltimateLink/build")
set(ELIS_LINK_LIB_HINTS "${ELIS_LINK_BUILD_DIR}")
if(ELIS_LINK_ROOT_DIR)
  list(APPEND ELIS_LINK_LIB_HINTS "${ELIS_LINK_ROOT_DIR}/build")
endif()
find_library(ELIS_LINK_CORE_LIB NAMES elis_link_core PATHS ${ELIS_LINK_LIB_HINTS})
if(ELIS_LINK_CORE_LIB AND ELIS_LINK_INCLUDE_DIR)
  target_link_libraries(${PROJECT_NAME} ${ELIS_LINK_CORE_LIB})
  target_compile_definitions(${PROJECT_NAME} PRIVATE ELIS_LINK_ENABLED)
endif()

##########
# GTESTS #
##########
catkin_add_gtest(test_data_provider test/test_data_provider.cpp)
target_link_libraries(test_data_provider ${PROJECT_NAME})

catkin_add_gtest(test_camera_imu_synchronizer test/test_camera_imu_synchronizer.cpp)
target_link_libraries(test_camera_imu_synchronizer ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
