cmake_minimum_required(VERSION 3.0.2)
project(ze_vio_frontend)

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
  include/ze/vio_frontend/elis_link_handles.hpp
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
  src/elis_link_handles.cpp
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

cs_add_library(${PROJECT_NAME} SHARED ${SOURCES} ${HEADERS})

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
  # Hint Metavision SDK location (core config under /usr/share/cmake/MetavisionSDK and libs under /usr/lib)
  list(APPEND CMAKE_PREFIX_PATH "/usr/share/cmake/MetavisionSDK")
  list(APPEND CMAKE_PREFIX_PATH "/usr/share/cmake")
  list(APPEND CMAKE_LIBRARY_PATH "/usr/lib")
  message(STATUS "MV: CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}")
  message(STATUS "MV: CMAKE_LIBRARY_PATH=${CMAKE_LIBRARY_PATH}")
  find_package(MetavisionSDK QUIET COMPONENTS core driver)
  if(TARGET Metavision::SDK::Core)
    message(STATUS "MV: Metavision::SDK::Core target is available.")
  else()
    message(STATUS "MV: Metavision::SDK::Core target is NOT available.")
  endif()
  if(MetavisionSDK_core_FOUND)
    message(STATUS "MV: MetavisionSDK core found: ${MetavisionSDK_core_FOUND}")
    message(STATUS "MV: MetavisionSDK_CORE_INCLUDE_DIRS=${MetavisionSDK_CORE_INCLUDE_DIRS}")
    message(STATUS "MV: MetavisionSDK_CORE_LIBRARIES=${MetavisionSDK_CORE_LIBRARIES}")
  else()
    message(STATUS "MV: MetavisionSDK core NOT found; configure will continue without RAW C++ loader.")
  endif()
  # Fallback manual library detection if the imported target is not exported.
  find_library(MV_CORE_LIB NAMES metavision_sdk_core
    HINTS /usr/lib /usr/lib64 /usr/lib/x86_64-linux-gnu /opt/prophesee /opt/prophesee/*/lib)
  find_library(MV_DRIVER_LIB NAMES metavision_sdk_driver
    HINTS /usr/lib /usr/lib64 /usr/lib/x86_64-linux-gnu /opt/prophesee /opt/prophesee/*/lib)
  find_path(MV_CORE_INCLUDE_DIR
    NAMES metavision/sdk/base/events/event_cd.h metavision/sdk/core/events/event_cd.h
    HINTS /usr/include /opt/prophesee /opt/prophesee/*/include)
  find_path(MV_DRIVER_INCLUDE_DIR
    NAMES metavision/sdk/driver/camera.h metavision/sdk/driver/cd.h metavision/sdk/driver/event_file_reader.h metavision/sdk/driver/raw_event_file_reader.h
    HINTS /usr/include /opt/prophesee /opt/prophesee/*/include)
  if(MV_CORE_LIB AND MV_CORE_INCLUDE_DIR)
    message(STATUS "MV: Found metavision_sdk_core manually at ${MV_CORE_LIB}")
    message(STATUS "MV: Found Metavision core includes at ${MV_CORE_INCLUDE_DIR}")
  endif()
  if(MV_DRIVER_LIB AND MV_DRIVER_INCLUDE_DIR)
    message(STATUS "MV: Found metavision_sdk_driver manually at ${MV_DRIVER_LIB}")
    message(STATUS "MV: Found Metavision driver includes at ${MV_DRIVER_INCLUDE_DIR}")
  endif()
  if(Python3_Development_FOUND)
    target_link_libraries(${PROJECT_NAME} Python3::Python)
  endif()
  if(pybind11_FOUND)
    target_link_libraries(${PROJECT_NAME} pybind11::embed)
  endif()
  target_link_libraries(${PROJECT_NAME} ${ELIS_LINK_CORE_LIB})
  target_compile_definitions(${PROJECT_NAME} PRIVATE ELIS_LINK_ENABLED)
else()
  message(STATUS "ElisUltimateLink core lib not found at ${ELIS_LINK_BUILD_DIR}; building without ELIS_LINK support.")
endif()

# Optional frontend sanity test when Elis link is available.
if(ELIS_LINK_CORE_LIB)
  enable_testing()
  find_package(GTest QUIET)
  if(GTest_FOUND)
    add_executable(test_elis_link_enabled test/test_elis_link_enabled.cpp)
    target_link_libraries(test_elis_link_enabled
      ${PROJECT_NAME}
      ${ELIS_LINK_CORE_LIB}
      ${catkin_LIBRARIES}
      GTest::GTest
      GTest::Main)
    if(pybind11_FOUND)
      target_link_libraries(test_elis_link_enabled pybind11::embed)
    endif()
    if(Python3_Development_FOUND)
      target_link_libraries(test_elis_link_enabled Python3::Python)
    endif()
    target_compile_definitions(test_elis_link_enabled PRIVATE ELIS_LINK_ENABLED)
    add_test(NAME elis_link_frontend_flag_test COMMAND test_elis_link_enabled)

    add_executable(test_elis_handle_continuity test/test_elis_handle_continuity.cpp)
    target_link_libraries(test_elis_handle_continuity
      ${PROJECT_NAME}
      ${ELIS_LINK_CORE_LIB}
      ${catkin_LIBRARIES}
      GTest::GTest
      GTest::Main)
    if(pybind11_FOUND)
      target_link_libraries(test_elis_handle_continuity pybind11::embed)
    endif()
    if(Python3_Development_FOUND)
      target_link_libraries(test_elis_handle_continuity Python3::Python)
    endif()
    target_compile_definitions(test_elis_handle_continuity PRIVATE ELIS_LINK_ENABLED)
    add_test(NAME elis_link_handle_continuity_test COMMAND test_elis_handle_continuity)
  else()
    message(STATUS "GTest not found; skipping test_elis_link_enabled")
  endif()

  set(HAVE_METAVISION_SDK OFF)
  set(MV_ALL_LIBS "")
  set(MV_ALL_INCLUDES "")
  # Prefer imported targets if they exist.
  if(MetavisionSDK_core_FOUND AND TARGET Metavision::SDK::Core)
    list(APPEND MV_ALL_LIBS Metavision::SDK::Core)
  elseif(MV_CORE_LIB AND MV_CORE_INCLUDE_DIR)
    list(APPEND MV_ALL_LIBS ${MV_CORE_LIB})
    list(APPEND MV_ALL_INCLUDES ${MV_CORE_INCLUDE_DIR})
  endif()
  if(MetavisionSDK_driver_FOUND AND TARGET Metavision::SDK::Driver)
    list(APPEND MV_ALL_LIBS Metavision::SDK::Driver)
  elseif(MV_DRIVER_LIB AND MV_DRIVER_INCLUDE_DIR)
    list(APPEND MV_ALL_LIBS ${MV_DRIVER_LIB})
    list(APPEND MV_ALL_INCLUDES ${MV_DRIVER_INCLUDE_DIR})
  endif()
  if(MV_ALL_LIBS)
    set(HAVE_METAVISION_SDK ON)
    message(STATUS "MV: Harness will use linkable Metavision libs: ${MV_ALL_LIBS}")
    if(MV_ALL_INCLUDES)
      message(STATUS "MV: Harness include dirs: ${MV_ALL_INCLUDES}")
    endif()
  else()
    message(STATUS "Metavision SDK (C++) not found; elis_link_harness will skip RAW loading via C++ API.")
  endif()

  add_executable(elis_link_harness src/elis_link_harness.cpp)
  target_link_libraries(elis_link_harness
    ${PROJECT_NAME}
    ${ELIS_LINK_CORE_LIB}
    ${catkin_LIBRARIES})
  if(HAVE_METAVISION_SDK)
    target_link_libraries(elis_link_harness ${MV_ALL_LIBS})
    target_include_directories(elis_link_harness PRIVATE ${MV_ALL_INCLUDES})
    target_compile_definitions(elis_link_harness PRIVATE HAVE_METAVISION_SDK)
  endif()
  target_compile_definitions(elis_link_harness PRIVATE ELIS_LINK_ENABLED)

  # Always build elis_link_playback; it will emit a clear runtime message if the
  # Metavision SDK is not available. This avoids stale binaries when toggling
  # SDK availability across builds.
  add_executable(elis_link_playback src/elis_link_playback.cpp)
  target_link_libraries(elis_link_playback
    ${PROJECT_NAME}
    ${ELIS_LINK_CORE_LIB}
    ${catkin_LIBRARIES})
  target_compile_definitions(elis_link_playback PRIVATE ELIS_LINK_ENABLED)
  if(HAVE_METAVISION_SDK)
    target_link_libraries(elis_link_playback ${MV_ALL_LIBS})
    if(MV_ALL_INCLUDES)
      target_include_directories(elis_link_playback PRIVATE ${MV_ALL_INCLUDES})
    endif()
    target_compile_definitions(elis_link_playback PRIVATE HAVE_METAVISION_SDK)
  endif()

  # Live camera runner (hardware). Requires USB passthrough + Metavision SDK at runtime.
  add_executable(elis_link_live src/elis_link_live.cpp)
  target_link_libraries(elis_link_live
    ${PROJECT_NAME}
    ${ELIS_LINK_CORE_LIB}
    ${catkin_LIBRARIES})
  target_compile_definitions(elis_link_live PRIVATE ELIS_LINK_ENABLED)
  if(HAVE_METAVISION_SDK)
    target_link_libraries(elis_link_live ${MV_ALL_LIBS})
    if(MV_ALL_INCLUDES)
      target_include_directories(elis_link_live PRIVATE ${MV_ALL_INCLUDES})
    endif()
    target_compile_definitions(elis_link_live PRIVATE HAVE_METAVISION_SDK)
  endif()

  # RT1060 telemetry runner (serial CDC). Uses ELIS link for keypoints.
  add_executable(elis_link_rt1060 src/elis_link_rt1060.cpp)
  target_link_libraries(elis_link_rt1060
    ${PROJECT_NAME}
    ${ELIS_LINK_CORE_LIB}
    ${catkin_LIBRARIES})
  target_compile_definitions(elis_link_rt1060 PRIVATE ELIS_LINK_ENABLED)
endif()

##########
# EXPORT #
##########
cs_install()
cs_export()
