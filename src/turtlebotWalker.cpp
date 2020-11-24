
/**
 * @copyright (c) 2020, Kartik Venkat
 *
 * @file turtlebotWalker.cpp
 * @authors
 * Kartik Venkat kartik.venkat86@gmail.com \n
 * @version 1.0
 *
 * @section LICENSE
 *
 * MIT License
 * @section DESCRIPTION:
 * This is the implementation for the turtlebot walker node.
 */

#include <iostream>
#include "../include/turtlebotWalker.hpp"


/*
         * @brief class constructor.
         */
TurtlebotWalker::TurtlebotWalker() {
    pubVel = nh.advertise <geometry_msgs::Twist>
            ("/cmd_vel", 1000);
    sub = nh.subscribe<sensor_msgs::LaserScan>
            ("/scan", 1000,
             &TurtlebotWalker::scanEnvCallback, this);

    msg.linear.x = 0.0;
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = 0.0;

    isObstacle = false;
    obstacleThresh = 0.5;
    pubVel.publish(msg);
}
/*
         * @brief Callback service to scan the environment for obstacles.
         * @param msg : Pointer for messages from the LaserScan sensor.
         * @return None.
         */
void TurtlebotWalker::scanEnvCallback(const sensor_msgs::
                                    LaserScan::ConstPtr& msg) {
    for ( auto i : msg->ranges ) {
        if ( i < 1.0 ) {
            isObstacle = true;
            return;
        } else {
            isObstacle = false;
            return;
        }
    }
}
/*
         * @brief Detect obstacles in the environment.
         * @param None
         * @return bool value determining if obstacle is detected.
         */
bool TurtlebotWalker::checkObstacle() {
    return isObstacle;
}
/*
         * @brief Function that moves the TurtleBot in the environment.
         * @param None.
         * @return None.
         */
void TurtlebotWalker::moveRobot() {
    ros::Rate loop(10);

    while (ros::ok()) {
        if (checkObstacle()) {
            ROS_WARN_STREAM("Obstacle Detected Ahead !!!");
            msg.linear.x = 0.0;
            msg.angular.z = 1.0;
        } else {
            ROS_INFO_STREAM("The path ahead is clear. Moving Forward !!");
            msg.linear.x = 0.75;
            msg.angular.z = 0.0;
        }
        pubVel.publish(msg);
        ros::spinOnce();
        loop.sleep();
    }
}
/*
         * @brief class destructor.
         */
TurtlebotWalker::~TurtlebotWalker() {
    msg.linear.x = 0.0;
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = 0.0;
    pubVel.publish(msg);
}

