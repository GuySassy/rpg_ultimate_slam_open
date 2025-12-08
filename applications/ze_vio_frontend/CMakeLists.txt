project(ze_vio_frontend)
cmake_minimum_required(VERSION 2.8.3)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

catkin_python_setup()

include(ze_setup)

# Optional ElisUltimateLink linkage
set(ELIS_LINK_ROOT_DIR "${ELIS_LINK_ROOT_DIR}" CACHE PATH "Path to elisUltimateLink root")

set(ELIS_LINK_INCLUDE_HINTS
  "${PROJECT_SOURCE_DIR}/../elisUltimateLink/include"
  "${PROJECT_SOURCE_DIR}/../../../../third_party/elisUltimateLink/include"
)
if(ELIS_LINK_ROOT_DIR)
  list(APPEND ELIS_LINK_INCLUDE_HINTS "${ELIS_LINK_ROOT_DIR}/include")
endif()

find_path(ELIS_LINK_INCLUDE_DIR elisUltimateLink/keypoint_service.hpp
  PATHS ${ELIS_LINK_INCLUDE_HINTS}
)
if(ELIS_LINK_INCLUDE_DIR)
  message(STATUS "ElisUltimateLink include dir: ${ELIS_LINK_INCLUDE_DIR}")
  include_directories(${ELIS_LINK_INCLUDE_DIR})
endif()

#############
# LIBRARIES #
#############
set(HEADERS
  include/ze/vio_frontend/feature_initializer.hpp
  include/ze/vio_frontend/feature_tracker.hpp
  include/ze/vio_frontend/frontend_api.hpp
  include/ze/vio_frontend/frontend_base.hpp
  include/ze/vio_frontend/frontend_gflags.hpp
  include/ze/vio_frontend/frontend_vio.hpp
  include/ze/vio_frontend/keyframe_selection.hpp
  include/ze/vio_frontend/landmarks_reprojector.hpp
  include/ze/vio_frontend/refinement.hpp
  include/ze/vio_frontend/stereo_matcher.hpp
  include/ze/vio_frontend/track_extractor.hpp
  )

set(SOURCES
  src/feature_initializer.cpp
  src/feature_tracker.cpp
  src/frontend_api.cpp
  src/frontend_base.cpp
  src/frontend_gflags.cpp
  src/frontend_logging.cpp
  src/frontend_vio.cpp
  src/keyframe_selection.cpp
  src/landmarks_reprojector.cpp
  src/refinement.cpp
  src/stereo_matcher.cpp
  src/track_extractor.cpp
  )

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})

# Optional: link ElisUltimateLink core when its build outputs are available.
# Look for a prebuilt libelis_link_core in the sibling elisUltimateLink build dir.
set(ELIS_LINK_BUILD_DIR "${PROJECT_SOURCE_DIR}/../elisUltimateLink/build")
set(ELIS_LINK_LIB_HINTS
  "${ELIS_LINK_BUILD_DIR}"
  "${PROJECT_SOURCE_DIR}/../../../../third_party/elisUltimateLink/build"
)
if(ELIS_LINK_ROOT_DIR)
  list(APPEND ELIS_LINK_LIB_HINTS "${ELIS_LINK_ROOT_DIR}/build")
endif()

find_library(ELIS_LINK_CORE_LIB NAMES elis_link_core PATHS ${ELIS_LINK_LIB_HINTS})
if(ELIS_LINK_CORE_LIB)
  message(STATUS "ElisUltimateLink core lib: ${ELIS_LINK_CORE_LIB}")
  find_package(Python3 COMPONENTS Development QUIET)
  find_package(pybind11 QUIET)
  if(Python3_Development_FOUND)
    target_link_libraries(${PROJECT_NAME} PRIVATE Python3::Python)
  endif()
  if(pybind11_FOUND)
    target_link_libraries(${PROJECT_NAME} PRIVATE pybind11::embed)
  endif()
  target_link_libraries(${PROJECT_NAME} PRIVATE ${ELIS_LINK_CORE_LIB})
  target_compile_definitions(${PROJECT_NAME} PRIVATE ELIS_LINK_ENABLED)
else()
  message(STATUS "ElisUltimateLink core lib not found at ${ELIS_LINK_BUILD_DIR}; building without ELIS_LINK support.")
endif()

##########
# EXPORT #
##########
cs_install()
cs_export()
