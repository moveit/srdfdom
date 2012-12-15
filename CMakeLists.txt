cmake_minimum_required(VERSION 2.8.3)
project(srdfdom)

include_directories(include)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
find_package(tinyxml REQUIRED)

include_directories(SYSTEM ${tinyxml_INCLUDE_DIRS})
link_directories(${tinyxml_LIBRARY_DIRS})

find_package(Boost REQUIRED)
include_directories(SYSTEM ${Boost_INCLUDE_DIR})

find_package(catkin REQUIRED COMPONENTS console_bridge urdfdom_headers)

include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  INCLUDE_DIRS include
  DEPENDS console_bridge urdfdom_headers
)

add_library(${PROJECT_NAME} src/model.cpp)
target_link_libraries(${PROJECT_NAME} ${tinyxml_LIBRARIES} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
