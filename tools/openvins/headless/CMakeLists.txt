cmake_minimum_required(VERSION 3.10)
project(rm27_openvins_headless LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(OPENVINS_SOURCE "" CACHE PATH "Prepared, pinned OpenVINS source")
if(NOT EXISTS "${OPENVINS_SOURCE}/ov_msckf/src/core/VioManager.cpp")
  message(FATAL_ERROR "Set OPENVINS_SOURCE to the source prepared by build.py")
endif()
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs video features2d calib3d)
find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time)
find_package(Ceres REQUIRED)
if(CMAKE_CROSSCOMPILING AND CMAKE_SYSROOT AND TARGET ceres)
  # Ceres 1.13 exports host-absolute target-library paths. Relocate those
  # dependency references into the explicitly selected target sysroot.
  get_target_property(CERES_TARGET_DEPS ceres IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE)
  if(CERES_TARGET_DEPS)
    set(CERES_TARGET_RELOCATED "")
    foreach(DEPENDENCY IN LISTS CERES_TARGET_DEPS)
      if(DEPENDENCY MATCHES "^/usr/")
        if(NOT EXISTS "${CMAKE_SYSROOT}${DEPENDENCY}")
          message(FATAL_ERROR "Missing target Ceres dependency: ${CMAKE_SYSROOT}${DEPENDENCY}")
        endif()
        list(APPEND CERES_TARGET_RELOCATED "${CMAKE_SYSROOT}${DEPENDENCY}")
      else()
        list(APPEND CERES_TARGET_RELOCATED "${DEPENDENCY}")
      endif()
    endforeach()
    set_target_properties(ceres PROPERTIES IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "${CERES_TARGET_RELOCATED}")
  endif()
endif()
find_package(Threads REQUIRED)
file(GLOB_RECURSE OV_CORE "${OPENVINS_SOURCE}/ov_core/src/*.cpp")
file(GLOB_RECURSE OV_INIT "${OPENVINS_SOURCE}/ov_init/src/*.cpp")
list(FILTER OV_CORE EXCLUDE REGEX "/test_[^/]*\\.cpp$")
list(FILTER OV_INIT EXCLUDE REGEX "/(test_[^/]*|SimulatorInit)\\.cpp$")
set(OV_MSCKF
  state/State.cpp state/StateHelper.cpp state/Propagator.cpp
  core/VioManager.cpp core/VioManagerHelper.cpp
  update/UpdaterHelper.cpp update/UpdaterMSCKF.cpp update/UpdaterSLAM.cpp
  update/UpdaterZeroVelocity.cpp)
set(OV_MSCKF_PATHS "")
foreach(OV_SOURCE IN LISTS OV_MSCKF)
  list(APPEND OV_MSCKF_PATHS "${OPENVINS_SOURCE}/ov_msckf/src/${OV_SOURCE}")
endforeach()
add_library(openvins_estimator STATIC ${OV_CORE} ${OV_INIT} ${OV_MSCKF_PATHS} audit.cpp)
target_compile_definitions(openvins_estimator PUBLIC ROS_AVAILABLE=0 ENABLE_ARUCO_TAGS=0)
target_include_directories(openvins_estimator PUBLIC
  "${OPENVINS_SOURCE}/ov_core/src" "${OPENVINS_SOURCE}/ov_init/src"
  "${OPENVINS_SOURCE}/ov_msckf/src" "${CMAKE_CURRENT_SOURCE_DIR}"
  ${EIGEN3_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})
target_link_libraries(openvins_estimator PUBLIC ${OpenCV_LIBS} ${Boost_LIBRARIES}
  ${CERES_LIBRARIES} Threads::Threads)
add_executable(openvins_headless main.cpp)
target_link_libraries(openvins_headless PRIVATE openvins_estimator)
install(TARGETS openvins_headless RUNTIME DESTINATION bin)
