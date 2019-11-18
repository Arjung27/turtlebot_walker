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
 *@file walkerAlgorithm.cpp
 *@author Arjun Gupta
 *@copyright MIT License
 *@brief define and implement walkerAlgorithm class functions
 */

#include <iostream>
#include "../include/turtlebot_walker/walkerAlgorithm.hpp"

walkerAlgorithm::walkerAlgorithm() {
    // Default value for linearVelocity and angularVelocity
    linearVelocity = 0.3;
    angularVelocity = -1.0;
    collision = false;
    ROS_INFO_STREAM("Default parameters parameters initialized");
    // Publish on the topic
    publishVelocity = nh.advertise<geometry_msgs::Twist>(\
                            "cmd_vel_mux/input/navi", 1000);
    // Subscribe to the topic
    subscribeVelocity = nh.subscribe<sensor_msgs::LaserScan>(\
                    "/scan", 1000, &walkerAlgorithm::laserScanCallback, this);
    // Initialize initial linear and angular velocities
    msg.linear.x = 0.0;
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = 0.0;
    publishVelocity.publish(msg);
}

walkerAlgorithm::~walkerAlgorithm() {
    ROS_WARN_STREAM("Stopping the bot");
    // Setting the velocity to zero to stop the bot
    msg.linear.x = 0.0;
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = 0.0;
    publishVelocity.publish(msg);
}

void walkerAlgorithm::laserScanCallback(\
                const sensor_msgs::LaserScan::ConstPtr& msg) {
    for (auto msgs_ : msg->ranges) {
        // check if any object is witihin 0.7 range
        if (msgs_ < 0.7) {
            collision = true;
            return;
        }
    }
    collision = false;
}

bool walkerAlgorithm::checkObstacle() {
    // return the collision value
    return collision;
}

void walkerAlgorithm::moveRobot() {
    // Set the ros loop rate
    ros::Rate loop_rate(5);
    // Run until ros runs
    while(ros::ok()) {
        // Check if the obstacle is detected or not
        if (checkObstacle()) {
            ROS_WARN_STREAM("Obstacle ahead within the range of 0.7");
            // Setting the linear velocity to be 0
            msg.linear.x = 0.0;
            // Changing the angular velocity for turning
            msg.angular.z = angularVelocity;
        } else {
            ROS_INFO_STREAM("No obstacle detected moving forward");
            // Stop the angular movement
            msg.angular.z = 0;
            // Set the linear velocity to the default value
            msg.linear.x = linearVelocity;
        }
        // Publish the current velocities
        publishVelocity.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
}
