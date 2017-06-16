cmake_minimum_required(VERSION 2.8.3)
project(microepsilon_scancontrol)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs std_srvs std_msgs)

find_package(Boost REQUIRED)

find_package (PkgConfig REQUIRED)
pkg_check_modules (ARAVIS4 REQUIRED aravis-0.4)


### BUILD ###
include_directories(common/include /usr/local/include/libllt  /usr/local/include/libmescan  ${ARAVIS4_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
FIND_LIBRARY(LLT_LIBRARY  llt)
FIND_LIBRARY(MESCAN_LIBRARY  mescan)



catkin_package(
   INCLUDE_DIRS common/include /usr/local/include/libllt  /usr/local/include/libmescan
   LIBRARIES ${PROJECT_NAME} ${LLT_LIBRARY} ${MESCAN_LIBRARY}
   CATKIN_DEPENDS roscpp sensor_msgs std_srvs std_msgs
   DEPENDS Boost ARAVIS4)

add_library(${PROJECT_NAME} common/src/${PROJECT_NAME}.cpp)
add_executable(${PROJECT_NAME}_node ros/src/${PROJECT_NAME}_node.cpp)


add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${LLT_LIBRARY} ${MESCAN_LIBRARY} ${ARAVIS4_LIBRARIES} )
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${catkin_LIBRARIES})


### INSTALL ###
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY common/include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)
