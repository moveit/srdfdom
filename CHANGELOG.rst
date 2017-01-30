^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package srdfdom
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.2 (2017-01-30)
------------------
* [fix] gcc6 build error `#28 <https://github.com/ros-planning/srdfdom/issues/28>`_
* [fix] Compile with -std=c++11 (`#29 <https://github.com/ros-planning/srdfdom/issues/29>`_)
* [enhancement] cleanup urdfdom compatibility (`#27 <https://github.com/ros-planning/srdfdom/issues/27>`_)
* Contributors: Dmitry Rozhkov, Isaac I.Y. Saito, Robert Haschke, Victor Matare

0.4.1 (2016-09-22)
------------------
* [fix][system] Build failure for Ubuntu Wily and Debian Jesie (urdfdom compatibility `#25 <https://github.com/ros-planning/srdfdom/issues/25>`_)
  * test for existence of urdf typedef
  * if not existing, activate compatibility header
* Contributors: Michael Goerner, Robert Haschke

0.4.0 (2016-09-09)
------------------
* [fix] Define shared_ptr typedef (adjusting to the recent change in urdfdom) `#21 <https://github.com/ros-planning/srdfdom/issues/21>`_
* Contributors: Dave Coleman, Robert Haschke

0.3.2 (2016-08-25)
------------------
* [feat] Move SRDF-specific commands from moveit package `#14 <https://github.com/ros-planning/srdfdom/issues/14>`_
* [sys] remove ROS-dependent logging.
* [sys] Much cleanup in package.xml. `#12 <https://github.com/ros-planning/srdfdom/issues/12>`_ pkg-config is no longer used after https://github.com/ros-planning/srdfdom/commit/19b23e5900e9c179089e99caae52023f95d2fec8#diff-af3b638bc2a3e6c650974192a53c7291
* Contributors: Dave Coleman, Sarah Elliott, Robert Haschke, Isaac I.Y. Saito

0.3.1 (2016-08-01)
------------------
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
