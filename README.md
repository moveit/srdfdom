srdfdom
=======

Parser for the Semantic Robot Description Format (SRDF).

Includes a cpp and a python parser, as well as a cpp writer.

## Travis - Continuous Integration

Indigo | Kinetic
------ | -------
[![Build Status](https://travis-ci.org/ros-planning/srdfdom.svg?branch=indigo-devel)](https://travis-ci.org/ros-planning/srdfdom) | [![Build Status](https://travis-ci.org/ros-planning/srdfdom.svg?branch=kinetic-devel)](https://travis-ci.org/ros-planning/srdfdom)

## Authors

Original reflection implementation for SDF and URDF.

* Thomas Moulard - `urdfpy` implementation, integration
* David Lu - `urdf_python` implementation, integration
* Kelsey Hawkins - `urdf_parser_python` implementation, integration
* Antonio El Khoury - bugfixes
* Eric Cousineau - reflection (serialization?) changes
Reused for srdf python parser
* Guillaume Walck - `srdfpy` conversion, integration
* Dave Coleman - `srdf_writer.cpp` implementation

## C++ example

test_parser.cpp contains examples how to access the srdf elements from the cpp parser

## Python example

test.py contains examples how to access the srdf elements from the python parser

display_srdf reads a srdf from a file given in command line argument
or from parameter server (robot_description_semantic) and displays it in a yaml format
if an output option (-o <filename>) is provided, dumps the xml (re-generated from parsed input xml)

example:

    rosrun srdfdom display_srdf test/res/pr2_desc.3.srdf

## Test

    catkin_make run_tests_srdfdom
