cmake_minimum_required( VERSION 2.8.3 )
project (srdfdom CXX C)

# set the default build type
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

include_directories(include)

find_library(tinyxml_library tinyxml)
if (tinyxml_library)
  message (STATUS "Looking for libtinyxml - found")
  set(tinyxml_libraries ${tinyxml_library})
endif ()
find_path(tinyxml_include_dirs NAMES tinyxml.h PATH_SUFFIXES tinyxml)
if (NOT tinyxml_include_dirs)
   message (STATUS "Looking for tinyxml/tinyxml.hpp or tinyxml/tinyxml.h - not found.")
endif ()
if (NOT tinyxml_include_dirs OR NOT tinyxml_libraries)
   include (FindPkgConfig)
   if (PKG_CONFIG_FOUND)
     # Find tinyxml
     pkg_check_modules(tinyxml tinyxml)
   else()
     MESSAGE("Missing: tinyxml")
   endif()
endif ()
include_directories(SYSTEM ${tinyxml_include_dirs})
link_directories(${tinyxml_library_dirs})
add_definitions(${tinyxml_cflags})

find_package(Boost REQUIRED)
include_directories(SYSTEM ${Boost_INCLUDE_DIR})

find_package(catkin REQUIRED COMPONENTS urdfdom_headers console_bridge)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  INCLUDE_DIRS include
  DEPENDS urdfdom_headers)

add_library(${PROJECT_NAME} SHARED src/model.cpp)
target_link_libraries(${PROJECT_NAME} ${tinyxml_libraries} ${catkin_LIBRARIES})
install(TARGETS ${PROJECT_NAME} DESTINATION lib/)
install(DIRECTORY include/srdfdom DESTINATION include/)

find_package(urdfdom QUIET)
if (urdfdom_FOUND)
  include_directories(SYSTEM ${urdfdom_INCLUDE_DIRS})
  add_definitions('-DTEST_RESOURCE_LOCATION="${CMAKE_CURRENT_SOURCE_DIR}/test/res/"')
  add_executable(test_srdf test/test_parser.cpp)
  target_link_libraries(test_srdf ${PROJECT_NAME} ${urdfdom_LIBRARIES})
else()
  message(STATUS "Tests are not built without urdfdom")
endif()
