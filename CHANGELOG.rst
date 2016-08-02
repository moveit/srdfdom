^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package srdfdom
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Change logError to Warn if collision link missing `#10 <https://github.com/ros-planning/srdfdom/issues/10>`_ Since MoveIt continues to load anyway, it makes sense to change the unknown collision link pairs ROS Error to a ROS Warning. Everything continues to work if a specified set of collision-link pairs is missing.
* Contributors: Dave Coleman, Ian McMahon

0.3.0 (2015-06-16)
------------------
* Removed unwanted python compiled file
* Fixed path to resource in python test to work for rostest
* Fixed authors, added doc
* Fixed group_state parsing and changed chain as an aggregate
* Renamed groups as subgroups when integrated in a group
* Added the cpp tests in the python test
* Fixed missing install
* Added a python parser based on urdf_parser_py and using its reflection interface
* Contributors: Dave Coleman, Guillaume Walck

0.2.7 (2014-07-01)
------------------
* fixing dependencies for https://github.com/ros/rosdistro/issues/4633
* added travis build status indicator in README.md
* added travis support
* use FindTinyXML from cmake_module
* Contributors: Dave Coleman, Dave Hershberger, Ioan Sucan, Tully Foote

0.2.6 (2013-07-19)
------------------
* fix incorrect tag name
