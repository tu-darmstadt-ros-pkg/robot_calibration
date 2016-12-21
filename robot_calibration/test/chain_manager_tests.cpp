/*
 * Copyright (C) 2014-2015 Fetch Robotics Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <robot_calibration/capture/chain_manager.h>
#include <gtest/gtest.h>

TEST(ChainManagerTests, test_rosparam_loading)
{
  ros::NodeHandle nh("~");
  robot_calibration::ChainManager manager(nh, 0.001);

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "chain_manager_tests");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
