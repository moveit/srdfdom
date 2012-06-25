cmake_minimum_required( VERSION 2.8 FATAL_ERROR )
project (srdfdom CXX C)

set (SRDF_MAJOR_VERSION 0)
set (SRDF_MINOR_VERSION 1)
set (SRDF_PATCH_VERSION 0)

set (SRDF_VERSION ${SRDF_MAJOR_VERSION}.${SRDF_MINOR_VERSION}.${SRDF_PATCH_VERSION})

message (STATUS "${PROJECT_NAME} version ${SRDF_VERSION}")

# set the default build type
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# This shouldn't be necessary, but there has been trouble
# with MSVC being set off, but MSVCXX ON.
if(MSVC OR MSVC90 OR MSVC10)
  set(MSVC ON)
endif (MSVC OR MSVC90 OR MSVC10)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include_directories(include)

include(SearchForTinyXML)
include_directories(SYSTEM ${tinyxml_include_dirs})
link_directories(${tinyxml_library_dirs})
add_definitions(${tinyxml_cflags})

find_package(urdfdom_headers REQUIRED)
include_directories(SYSTEM ${urdfdom_headers_INCLUDE_DIRS})

find_package(Boost REQUIRED)
include_directories(SYSTEM ${Boost_INCLUDE_DIR})

find_package(console_bridge REQUIRED)
include_directories(SYSTEM ${console_bridge_INCLUDE_DIRS})

add_library(${PROJECT_NAME} SHARED src/model.cpp)
target_link_libraries(${PROJECT_NAME} ${tinyxml_libraries} ${console_bridge_LIBRARIES})
install(TARGETS ${PROJECT_NAME} DESTINATION lib/)
install(DIRECTORY include/srdfdom DESTINATION include/)

find_package(urdfdom QUIET)
if (urdfdom_FOUND)
  enable_testing()
  include_directories(SYSTEM ${urdfdom_INCLUDE_DIRS})
  add_definitions('-DTEST_RESOURCE_LOCATION="${CMAKE_CURRENT_SOURCE_DIR}/test/res/"')
  add_executable(test_srdf test/test_parser.cpp)
  target_link_libraries(test_srdf ${PROJECT_NAME} ${urdfdom_LIBRARIES})
  add_test(test_srdf ${EXECUTABLE_OUTPUT_PATH}/test_srdf)
else()
  message(STATUS "Tests are not built without urdfdom")
endif()

set(PKG_NAME ${PROJECT_NAME})
set(PKG_LIBRARIES ${PROJECT_NAME})
set(PKG_DEPENDS urdfdom_headers)
set(cmake_conf_file "${CMAKE_CURRENT_SOURCE_DIR}/cmake/srdfdom-config.cmake")
configure_file("${cmake_conf_file}.in" "${cmake_conf_file}" @ONLY)
install(FILES ${cmake_conf_file} DESTINATION share/${PROJECT_NAME}/cmake/ COMPONENT cmake)

# Make the package config file
if (NOT MSVC)
  set(PKG_DESC "Semantic Robot Description Format")
  set(PKG_SRDF_LIBS "-l${PROJECT_NAME}")
  set(pkg_conf_file "${CMAKE_CURRENT_SOURCE_DIR}/cmake/pkgconfig/srdfdom.pc")
  configure_file("${pkg_conf_file}.in" "${pkg_conf_file}" @ONLY)
  install(FILES ${pkg_conf_file} DESTINATION lib/pkgconfig/ COMPONENT pkgconfig)
endif()
