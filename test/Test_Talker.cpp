/**
 * @file Test_Talker.cpp
 * @brief This File contains the code for testing the talker node.
 *
 * @author Vaibhav Bhilare
 * @copyright 2017, Vaibhav Bhilare
 *
 * MIT License
 * Copyright (c) 2017 Vaibhav Bhilare
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/* --Includes-- */
#include <ros/ros.h>
#include <ros/service_client.h>
#include <tf/transform_listener.h>
#include <gtest/gtest.h>
#include "beginner_tutorials/String_Modify.h"

/**
 * @brief A simple test to check the existence of the service
 * @param TESTSuite                    gtest framework
 * @param Service_Existence_Test       Test name
 */

TEST(TESTSuite, Service_Existance_Test) {
  // ROS Node handle Creation
  ros::NodeHandle n;

  // Client to the service String_Modify
  ros::ServiceClient client =
      n.serviceClient<beginner_tutorials::String_Modify>("String_Modify");

  // Check the Existence of Service
  bool exists(client.waitForExistence(ros::Duration(10)));
  EXPECT_TRUE(exists);
}

/**
 * @brief A simple test to check the Message Modification by service
 * @param TESTSuite                    gtest framework
 * @param String_Modification_Test     Test name
 */

TEST(TESTSuite, String_Modification_Test) {
  // ROS Node handle Creation
  ros::NodeHandle n;

  // Client to the service String_Modify
  ros::ServiceClient client =
      n.serviceClient<beginner_tutorials::String_Modify>("String_Modify");

  beginner_tutorials::String_Modify srv;
  srv.request.Message = "test";

  // Call the Service Client
  client.call(srv);
  EXPECT_STREQ("test", srv.response.MessageResponse.c_str());
}

