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
find_package(tinyxml_vendor REQUIRED)

include_directories(include ${rclcpp_INCLUDE_DIRS}
  ${rmw_implementation_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${TinyXML_INCLUDE_DIRS}
  ${console_bridge_INCLUDE_DIRS}
  ${urdfdom_headers_INCLUDE_DIRS}
  ${urdf_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  src/model.cpp
  src/srdf_writer.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  ${TinyXML_LIBRARIES}
  ${console_bridge_LIBRARIES}
  urdfdom_headers
  urdf
)

ament_export_libraries(${PROJECT_NAME})
ament_export_include_directories(include)

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


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_parser test/srdf_parser.test)

  add_definitions(-DTEST_RESOURCE_LOCATION="${CMAKE_SOURCE_DIR}/test/resources")
  execute_process(COMMAND bash -c "locale -a | grep -q ^nl_NL"
    RESULT_VARIABLE TEST_LOCALE
    OUTPUT_QUIET ERROR_QUIET)
  if (TEST_LOCALE)
    message(WARNING "Locale nl_NL not available. Locale test will not be meaningful.")
  endif()

  add_rostest_gtest(test_cpp test/srdf_parser_cpp.test test/test_parser.cpp)
  target_link_libraries(test_cpp ${catkin_LIBRARIES} ${PROJECT_NAME})
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(tinyxml_vendor)
ament_export_dependencies(console_bridge)
ament_export_dependencies(urdfdom_headers)
ament_export_dependencies(urdf)
ament_package()
