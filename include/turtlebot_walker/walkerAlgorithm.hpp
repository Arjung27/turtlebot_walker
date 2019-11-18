/**
 *  MIT License
 *
 *  Copyright (c) 2019 Arjun Gupta
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a
 *  copy of this software and associated documentation files (the "Software"),
 *  to deal in the Software without restriction, including without
 *  limitation the rights to use, copy, modify, merge, publish, distribute,
 *  sublicense, and/or sell copies of the Software, and to permit persons to
 *  whom the Software is furnished to do so, subject to the following
 *  conditions:
 *
 *  The above copyright notice and this permission notice shall be included
 *  in all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 *  THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 *  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 *  DEALINGS IN THE SOFTWARE.
 */

/**
 *@file walkerAlgorithm.hpp
 *@author Arjun Gupta
 *@copyright MIT License
 *@brief declare walkerAlgorithm class functions
 */

#ifndef INCLUDE_TURTLEBOT_WALKER_WALKERALGORITHM_HPP_
#define INCLUDE_TURTLEBOT_WALKER_WALKERALGORITHM_HPP_

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

/**
 * @brief      Class for walkerAlgorithm.
 */
class walkerAlgorithm {
 private:
  // Variable for velocities
  geometry_msgs::Twist msg;
  // node handler object
  ros::NodeHandle nh;
  // Publisher object to publish velocities
  ros::Publisher publishVelocity;
  // Subscriber object to subscribe laserscan topic
  ros::Subscriber subscribeVelocity;
  // variable for default linear speed
  float linearVelocity;
  // variable for default angular speed
  float angularVelocity;
  // Variable to detect collisions
  bool collision;

 public:
  /**
   * @brief  constructor for walkerAlgorithm
   */
  walkerAlgorithm();
  /**
   * @brief  default destructor to destroy object
   */
  ~walkerAlgorithm();
  /**
   * @brief  callback function to analyze laserScan data
   * 
   * @param  msg Pointer to messages from LaserScan
   * @return void
   */
  void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

  /**
   * @brief   Checks for obstacles nearby
   * @return  Boolean type value. 1 if obstacle is found else 0.
   */
  bool checkObstacle();
  /**
   * @brief  function to navigate the robot
   * @return void
   */
  void moveRobot();
};

#endif  // INCLUDE_TURTLEBOT_WALKER_WALKERALGORITHM_HPP_
