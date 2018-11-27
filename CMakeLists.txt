cmake_minimum_required(VERSION 2.8.3)
project(selected_pointcloud_publish_plugin)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rviz
  sensor_msgs
  )

# Use ccache if installed to make it fast to generate object files
find_program(CCACHE_FOUND ccache)
if(CCACHE_FOUND)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
endif(CCACHE_FOUND)
catkin_package(
    DEPENDS rviz sensor_msgs
    CATKIN_DEPENDS
    INCLUDE_DIRS
)


set(ROS_BUILD_TYPE Release)

include_directories(src ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

if(rviz_QT_VERSION VERSION_LESS "5")
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  include(${QT_USE_FILE})
else()
  find_package(Qt5Widgets REQUIRED)
endif()
add_definitions(-DQT_NO_KEYWORDS -g)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

#set(SOURCE_FILES src/ambient_sound_display.cpp src/ambient_sound_visual.cpp)

set(HEADER_FILES
  src/select_point_cloud_publish_action.h
)

if(rviz_QT_VERSION VERSION_LESS "5")
  qt4_wrap_cpp(MOC_FILES ${HEADER_FILES})
else()
  qt5_wrap_cpp(MOC_FILES ${HEADER_FILES})
endif()

set(SOURCE_FILES
  src/select_point_cloud_publish_action.cpp
  ${MOC_FILES}
)

add_library(selected_pointcloud_publish_plugin ${SOURCE_FILES} ${UIC_FILES})
if(rviz_QT_VERSION VERSION_LESS "5")
  target_link_libraries(selected_pointcloud_publish_plugin ${QT_LIBRARIES} ${catkin_LIBRARIES} ${rviz_DEFAULT_PLUGIN_LIBRARIES})
else()
  target_link_libraries(selected_pointcloud_publish_plugin Qt5::Widgets ${catkin_LIBRARIES} ${rviz_DEFAULT_PLUGIN_LIBRARIES})
endif()

if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
  set_target_properties(selected_pointcloud_publish_plugin PROPERTIES LINK_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -z defs")
endif()

install(TARGETS selected_pointcloud_publish_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY src/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
