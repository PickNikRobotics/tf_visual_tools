cmake_minimum_required(VERSION 2.8.3)
project(tf_visual_tools)

# C++ 11
set(CMAKE_CXX_FLAGS "-std=c++11 -Wall ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2
  tf2_msgs
  tf
  roslib
  roslint
  cmake_modules
  rosparam_shortcuts
  rviz_visual_tools
  tf_conversions
  interactive_markers
  rviz
  std_msgs
  geometry_msgs
)

find_package(Eigen3 REQUIRED)

# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS
  include
  LIBRARIES
  ${PROJECT_NAME}_gui
  CATKIN_DEPENDS
    message_runtime
    roscpp
    tf2
    tf
    rosparam_shortcuts
    interactive_markers
  DEPENDS
    EIGEN3
)

# Qt 4 or 5
if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} REQUIRED QtCore QtGui)
  include(${QT_USE_FILE})
  macro(qt_wrap_ui)
    qt4_wrap_ui(${ARGN})
  endmacro()
  macro(qt_wrap_cpp)
    qt4_wrap_cpp(${ARGN})
  endmacro()
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} REQUIRED Core Widgets)
  set(QT_LIBRARIES Qt5::Widgets)
  macro(qt_wrap_ui)
    qt5_wrap_ui(${ARGN})
  endmacro()
  macro(qt_wrap_cpp)
    qt5_wrap_cpp(${ARGN})
  endmacro()
endif()
## Prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
add_definitions(-DQT_NO_KEYWORDS)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIRS}
)

###########
## Build ##
###########

## specify which header files need to be run through "moc",
## Qt's meta-object compiler.
qt_wrap_cpp(MOC_FILES
  src/tf_visual_tools_gui.h
)

## specify the list of source files, including the output of
## the previous command which is stored in ``${MOC_FILES}``.
set(SOURCE_FILES
  src/tf_visual_tools_gui.cpp
  ${MOC_FILES}
)

add_library(${PROJECT_NAME}_receiver src/gui_remote_receiver.cpp)
target_link_libraries(${PROJECT_NAME}_receiver ${catkin_LIBRARIES})

add_library(${PROJECT_NAME}_gui ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME}_gui ${PROJECT_NAME}_receiver ${rviz_DEFAULT_PLUGIN_LIBRARIES} ${QT_LIBRARIES} ${catkin_LIBRARIES})

add_library(${PROJECT_NAME}_tf_manager
  src/tf_manager.cpp)
target_link_libraries(${PROJECT_NAME}_tf_manager
  ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_publisher
  src/tf_publisher.cpp)
target_link_libraries(${PROJECT_NAME}_publisher
  ${PROJECT_NAME}_tf_manager
  ${catkin_LIBRARIES})

#############
## Testing ##
#############

## Test for correct C++ source code
roslint_cpp()

#############
## Install ##
#############

# Install libraries
install(
  TARGETS
  ${PROJECT_NAME}_gui
  LIBRARY DESTINATION
  ${CATKIN_PACKAGE_LIB_DESTINATION}
  )

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# Install shared resources
install(DIRECTORY launch    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY resources DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# Install xml plugin config
install(FILES
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

# Install executables
install(TARGETS
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
