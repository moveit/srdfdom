cmake_minimum_required(VERSION 3.5)
project(srdfdom)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(console_bridge_vendor REQUIRED) # Provides console_bridge 0.4.0 on platforms without it.
find_package(console_bridge REQUIRED)
find_package(urdfdom_headers REQUIRED)
find_package(urdf REQUIRED)
find_package(tinyxml_vendor REQUIRED)
find_package(TinyXML REQUIRED)  # provided by tinyxml_vendor
find_package(ament_index_cpp REQUIRED)

find_package(Boost REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${rmw_implementation_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${TinyXML_INCLUDE_DIRS}
  ${console_bridge_INCLUDE_DIRS}
  ${urdfdom_headers_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
)

ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR src/${PROJECT_NAME})

add_library(${PROJECT_NAME}
  src/model.cpp
  src/srdf_writer.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  ${TinyXML_LIBRARIES}
  ${console_bridge_LIBRARIES}
  ${urdfdom_headers_LIBRARIES}
  ${urdf_LIBRARIES}
)

install(
  TARGETS
    ${PROJECT_NAME}
  DESTINATION lib
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

# Mark resources for installation
install(DIRECTORY test/resources
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/display_srdf
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_parser test/test_parser.cpp)
  target_link_libraries(test_parser
    ${PROJECT_NAME}
    ${rclcpp_LIBRARIES}
    ${TinyXML_LIBRARIES}
    ${console_bridge_LIBRARIES}
    ${urdfdom_headers_LIBRARIES}
    ${urdf_LIBRARIES}
    ament_index_cpp::ament_index_cpp
  )

  # python tests with python interfaces of message filters
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test.py "test/test.py"
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
    TIMEOUT 90)
endif()

ament_package()