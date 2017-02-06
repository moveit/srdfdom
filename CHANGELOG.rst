^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package srdfdom
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.5 (2017-02-06)
------------------
* [Indigo] cleanup urdfdom compatibility (cherry-picking `#27 <https://github.com/ros-planning/srdfdom/issues/27>`_) `#30 <https://github.com/ros-planning/srdfdom/issues/30>`_
* Contributors: Isaac I.Y. Saito, Michael Goerner, Robert Haschke

0.3.4 (2017-01-30)
------------------
* [maintenance] re-add boost/shared_ptr.hpp include (`#26 <https://github.com/ros-planning/srdfdom/issues/26>`_)
* Contributors: Michael Goerner

0.3.3 (2016-09-09)
------------------
* [fix] Define shared_ptr typedef (adjusting to the recent change in urdfdom) `#18 <https://github.com/ros-planning/srdfdom/issues/18>`_
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
