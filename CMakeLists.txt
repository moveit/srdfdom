cmake_minimum_required(VERSION 3.5)
project(srdfdom)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(Boost REQUIRED)
find_package(console_bridge REQUIRED)
find_package(urdfdom_headers REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
# find_package(cmake_modules REQUIRED)
# find_package(urdfdom_py REQUIRED)
find_package(tinyxml_vendor REQUIRED)

include_directories(include ${rclcpp_INCLUDE_DIRS}
                    ${rmw_implementation_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIR}
                    ${TinyXML_INCLUDE_DIRS}
                    ${console_bridge_INCLUDE_DIRS}
                    ${urdfdom_headers_INCLUDE_DIRS}
                    ${std_msgs_INCLUDE_DIRS})
# link_directories(${catkin_LIBRARY_DIRS})

add_library(${PROJECT_NAME}
  src/model.cpp
  src/srdf_writer.cpp
)
ament_target_dependencies(${PROJECT_NAME}
        ${TinyXML_LIBRARIES}
        ${console_bridge_LIBRARIES}
        urdfdom_headers
        urdf)
# target_link_libraries(${PROJECT_NAME} ${TinyXML_LIBRARIES} ${catkin_LIBRARIES} ${console_bridge_LIBRARIES} ${urdfdom_headers_LIBRARIES})


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

# if(BUILD_TESTING)
#   find_package(rostest REQUIRED)
#   add_rostest(test/srdf_parser.test)
# endif()

ament_package()
