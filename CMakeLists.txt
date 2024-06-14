cmake_minimum_required(VERSION 3.0.2)
project(zed2-no_cuda)

#cmake_policy(SET CMP0054 NEW)



# set(${PROJECT_NAME}_CXX_FLAGS "-std=c++14")
# set(CMAKE_CXX_FLAGS "${${PROJECT_NAME}_CXX_FLAGS} -Wall -pedantic -g")
# set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_CXX_FLAGS_RELEASE "-O2 -s -flto")

set(SRC_VIDEO
    ${PROJECT_SOURCE_DIR}/src/videocapture.cpp
)
set(SRC_SENSORS
    ${PROJECT_SOURCE_DIR}/src/sensorcapture.cpp
)

set(HEADERS_VIDEO
    # Base
    ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/videocapture.hpp
    
    # Defines
    ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/defines.hpp
    ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/videocapture_def.hpp
)
set(HEADERS_SENSORS
    # Base
    ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/sensorcapture.hpp

    # Defines
    ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/defines.hpp
    ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/sensorcapture_def.hpp
)


## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
)

## Generate added messages and services with any dependencies listed here
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs cv_bridge
)


## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)

list(INSERT CMAKE_MODULE_PATH 0 "${PROJECT_SOURCE_DIR}/cmake")
add_definitions(-DSENSORS_MOD_AVAILABLE)
find_package(HIDAPI REQUIRED)
include_directories(${HIDAPI_INCLUDE_DIRS})
set(SRC_FULL ${SRC_FULL} ${SRC_SENSORS})
set(HDR_FULL ${HDR_FULL} ${HEADERS_SENSORS})
set(DEP_LIBS ${DEP_LIBS}
    ${LibUSB_LIBRARIES}
    ${HIDAPI_LIBRARIES}
    pthread )

add_definitions(-DVIDEO_MOD_AVAILABLE)
find_package(LIBUSB 1 REQUIRED)
include_directories(${LibUSB_INCLUDE_DIRS})
set(SRC_FULL ${SRC_FULL} ${SRC_VIDEO})
set(HDR_FULL ${HDR_FULL} ${HEADERS_VIDEO})
set(DEP_LIBS ${DEP_LIBS}
     ${LibUSB_LIBRARIES}
     )

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}
)

add_library(EXT_LIB SHARED ${SRC_FULL} )
target_link_libraries( EXT_LIB  ${DEP_LIBS})

## Declare a C++ executable
add_executable(zed2_node src/zed2_node.cpp)

## Specify libraries to link executable against
target_link_libraries(zed2_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  EXT_LIB
)


# ## Mark executables and/or libraries for installation
install(TARGETS zed2_node
  # ARCHIVE DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  # LIBRARY DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# ## Mark cpp header files for installation
install(TARGETS EXT_LIB
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# ## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   zed2_node.launch
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )


