cmake_minimum_required(VERSION 2.8.3)
project(scan_26xx)

find_package(catkin REQUIRED COMPONENTS diagnostic_msgs roscpp sensor_msgs)

find_package(Boost REQUIRED)




catkin_package()

### BUILD ###
include_directories(common/include common/include/libllt common/include/libmescan  /usr/local/include/aravis-0.4 /usr/include/glib-2.0 /usr/lib/x86_64-linux-gnu/glib-2.0/include /usr/include/libxml2 ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
FIND_LIBRARY(ARAVIS_LIBRARY aravis-0.4 /usr/local/lib/)
FIND_LIBRARY(LLT_LIBRARY llt common/lib/libllt)
FIND_LIBRARY(MESCAN_LIBRARY mescan common/lib/libmescan/)

add_library(${PROJECT_NAME} common/src/scanner26xx.cpp)
add_executable(${PROJECT_NAME}_node ros/src/scan_26xx_node.cpp)


add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${LLT_LIBRARY} ${MESCAN_LIBRARY} ${ARAVIS_LIBRARY})

target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${catkin_LIBRARIES})


### INSTALL ###
install(TARGETS scan_26xx_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
