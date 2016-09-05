/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Ioan Sucan */

#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>
#include <fstream>
#include <stdexcept>
#include <boost/lexical_cast.hpp>

#define EXPECT_TRUE(arg) if (!(arg)) throw std::runtime_error("Assertion failed at line " + boost::lexical_cast<std::string>(__LINE__))

#ifndef TEST_RESOURCE_LOCATION
#  define TEST_RESOURCE_LOCATION "."
#endif

urdf::ModelInterfaceSharedPtr loadURDF(const std::string& filename)
{
  // get the entire file
  std::string xml_string;
  std::fstream xml_file(filename.c_str(), std::fstream::in);
  if (xml_file.is_open())
  {
    while (xml_file.good())
    {
      std::string line;
      std::getline( xml_file, line);
      xml_string += (line + "\n");
    }
    xml_file.close();
    return urdf::parseURDF(xml_string);
  }
  else
  {
    throw std::runtime_error("Could not open file " + filename + " for parsing.");
    return urdf::ModelInterfaceSharedPtr();
  }
}

void testSimple(void)
{
  srdf::Model s;
  urdf::ModelInterfaceSharedPtr u = loadURDF(std::string(TEST_RESOURCE_LOCATION) + "/pr2_desc.urdf");
  EXPECT_TRUE(u != NULL);
  
  EXPECT_TRUE(s.initFile(*u, std::string(TEST_RESOURCE_LOCATION) + "/pr2_desc.1.srdf"));
  EXPECT_TRUE(s.getVirtualJoints().size() == 0);
  EXPECT_TRUE(s.getGroups().size() == 0);
  EXPECT_TRUE(s.getGroupStates().size() == 0);
  EXPECT_TRUE(s.getDisabledCollisionPairs().empty());
  EXPECT_TRUE(s.getEndEffectors().size() == 0);
  
  EXPECT_TRUE(s.initFile(*u, std::string(TEST_RESOURCE_LOCATION) + "/pr2_desc.2.srdf"));
  EXPECT_TRUE(s.getVirtualJoints().size() == 1);
  EXPECT_TRUE(s.getGroups().size() == 1);
  EXPECT_TRUE(s.getGroupStates().size() == 0);
  EXPECT_TRUE(s.getDisabledCollisionPairs().empty());
  EXPECT_TRUE(s.getEndEffectors().size() == 0);
  
  EXPECT_TRUE(s.initFile(*u, std::string(TEST_RESOURCE_LOCATION) + "/pr2_desc.1.srdf"));
  EXPECT_TRUE(s.getVirtualJoints().size() == 0);
  EXPECT_TRUE(s.getGroups().size() == 0);
  EXPECT_TRUE(s.getGroupStates().size() == 0);
  EXPECT_TRUE(s.getDisabledCollisionPairs().empty());
  EXPECT_TRUE(s.getEndEffectors().size() == 0);
}

void testComplex(void)
{
  srdf::Model s;
  urdf::ModelInterfaceSharedPtr u = loadURDF(std::string(TEST_RESOURCE_LOCATION) + "/pr2_desc.urdf");
  EXPECT_TRUE(u != NULL);

  EXPECT_TRUE(s.initFile(*u, std::string(TEST_RESOURCE_LOCATION) + "/pr2_desc.3.srdf"));
  EXPECT_TRUE(s.getVirtualJoints().size() == 1);
  EXPECT_TRUE(s.getGroups().size() == 7);
  EXPECT_TRUE(s.getGroupStates().size() == 2);
  EXPECT_TRUE(s.getDisabledCollisionPairs().size() == 2);
  EXPECT_TRUE(s.getDisabledCollisionPairs()[0].reason_ == "adjacent");
  EXPECT_TRUE(s.getEndEffectors().size() == 2);
  
  EXPECT_TRUE(s.getVirtualJoints()[0].name_ == "world_joint");
  EXPECT_TRUE(s.getVirtualJoints()[0].type_ == "planar");
  for (std::size_t i = 0 ; i < s.getGroups().size() ; ++i)
  {
    if (s.getGroups()[i].name_ == "left_arm" || s.getGroups()[i].name_ == "right_arm")
      EXPECT_TRUE(s.getGroups()[i].chains_.size() == 1);
    if (s.getGroups()[i].name_ == "arms")
      EXPECT_TRUE(s.getGroups()[i].subgroups_.size() == 2);
    if (s.getGroups()[i].name_ == "base")
      EXPECT_TRUE(s.getGroups()[i].joints_.size() == 1);
    if (s.getGroups()[i].name_ == "l_end_effector" || s.getGroups()[i].name_ == "r_end_effector")
    {
      EXPECT_TRUE(s.getGroups()[i].links_.size() == 1);
      EXPECT_TRUE(s.getGroups()[i].joints_.size() == 9);
    }
    if (s.getGroups()[i].name_ == "whole_body")
    {
      EXPECT_TRUE(s.getGroups()[i].joints_.size() == 1);
      EXPECT_TRUE(s.getGroups()[i].subgroups_.size() == 2);
    }
  }
  int index = 0;
  if (s.getGroupStates()[0].group_ != "arms")
    index = 1;
  
  EXPECT_TRUE(s.getGroupStates()[index].group_ == "arms");
  EXPECT_TRUE(s.getGroupStates()[index].name_ == "tuck_arms");
  EXPECT_TRUE(s.getGroupStates()[1-index].group_ == "base");
  EXPECT_TRUE(s.getGroupStates()[1-index].name_ == "home");
  
  const std::vector<double> &v = s.getGroupStates()[index].joint_values_.find("l_shoulder_pan_joint")->second;
  EXPECT_TRUE(v.size() == 1u);
  EXPECT_TRUE(v[0] == 0.2);
  const std::vector<double> &w = s.getGroupStates()[1-index].joint_values_.find("world_joint")->second;
  EXPECT_TRUE(w.size() == 3u);
  EXPECT_TRUE(w[0] == 0.4);
  EXPECT_TRUE(w[1] == 0);
  EXPECT_TRUE(w[2] == -1);
  
  index = (s.getEndEffectors()[0].name_[0] == 'r') ? 0 : 1;
  EXPECT_TRUE(s.getEndEffectors()[index].name_ == "r_end_effector");
  EXPECT_TRUE(s.getEndEffectors()[index].component_group_ == "r_end_effector");
  EXPECT_TRUE(s.getEndEffectors()[index].parent_link_ == "r_wrist_roll_link");
}

int main(int argc, char **argv)
{
  testSimple();
  testComplex();
  return 0;
}
