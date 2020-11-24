
/**
 * @copyright (c) 2020, Kartik Venkat
 *
 * @file main.cpp
 * @authors
 * Kartik Venkat kartik.venkat86@gmail.com \n
 * @version 1.0
 *
 * @section LICENSE
 *
 * MIT License
 * @section DESCRIPTION:
 * This is the main file to run the Gazebo demo..
 */

#include "../include/turtlebotWalker.hpp"

/*
 * @brief main function to run Walker algorithm demo
 * @param argc
 * @param argv
 * @return None
 */

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "turtlebot_gazebo_walker");
    TurtlebotWalker walker;
    walker.moveRobot();
    return 0;
}

