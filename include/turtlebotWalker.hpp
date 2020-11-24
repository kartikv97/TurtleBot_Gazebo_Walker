/**
 * @copyright (c) 2020, Kartik Venkat
 *
 * @file turtlebotWalker.hpp
 * @authors
 * Kartik Venkat kartik.venkat86@gmail.com \n
 * @version 1.0
 *
 * @section LICENSE
 *
 * MIT License
 * @section DESCRIPTION:
 * This is the header file for the turtlebot walker node.
 */

#ifndef INCLUDE_TURTLEBOTWALKER_HPP_
#define INCLUDE_TURTLEBOTWALKER_HPP_


#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

/*
 * @brief Walker class for TurtleBot
 *
 */

class TurtlebotWalker {
 private:
        geometry_msgs::Twist msg;
        ros::NodeHandle nh;
        ros::Publisher pubVel;
        ros::Subscriber sub;
        bool isObstacle;
        double obstacleThresh;

 public:
        /*
         * @brief class constructor.
         */
        TurtlebotWalker();

        /*
         * @brief Callback service to scan the environment for obstacles.
         * @param msg : Pointer for messages from the LaserScan sensor.
         * @return None.
         */
        void scanEnvCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

        /*
         * @brief Detect obstacles in the environment.
         * @param None
         * @return bool value determining if obstacle is detected.
         */
        bool checkObstacle();

        /*
         * @brief Function that moves the TurtleBot in the environment.
         * @param None.
         * @return None.
         */
        void moveRobot();

        /*
         * @brief class destructor.
         */
        ~TurtlebotWalker();
};

#endif  // INCLUDE_TURTLEBOTWALKER_HPP_
