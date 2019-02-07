cmake_minimum_required(VERSION 3.5)
project(srdfdom)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(Boost REQUIRED)

find_package(console_bridge REQUIRED)
find_package(urdfdom_headers REQUIRED)

find_package(ament_cmake REQUIRED COMPONENTS cmake_modules urdf urdfdom_py)

find_package(tinyxml_vendor REQUIRED)

include_directories(include ${Boost_INCLUDE_DIR} ${TinyXML_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${console_bridge_INCLUDE_DIRS} ${urdfdom_headers_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_compile_options(-std=c++11)

# catkin_python_setup()

# catkin_package(
#   LIBRARIES ${PROJECT_NAME}
#   INCLUDE_DIRS include ${TinyXML_INCLUDE_DIRS}
#   DEPENDS console_bridge urdfdom_headers urdfdom_py
# )

add_library(${PROJECT_NAME}
  src/model.cpp
  src/srdf_writer.cpp
)
target_link_libraries(${PROJECT_NAME} ${TinyXML_LIBRARIES} ${catkin_LIBRARIES} ${console_bridge_LIBRARIES} ${urdfdom_headers_LIBRARIES})


install(TARGETS ${PROJECT_NAME}
  DESTINATION lib
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
)

install(PROGRAMS
  scripts/display_srdf
  DESTINATION lib/${PROJECT_NAME}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/srdf_parser.test)
endif()

ament_package()
