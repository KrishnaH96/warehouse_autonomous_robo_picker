#pragma once

#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"

#include "Context.hpp"

class Constants
{

public:
    /**
     * @brief The starting location for the pick and place task.
     */
    geometry_msgs::msg::PoseStamped startGoal;

    /**
     * @brief The goal location where the object should be placed.
     */
    geometry_msgs::msg::PoseStamped  endGoal;

    /**
     * @brief Get the Instance object
     * 
     * @return Constants& 
     */
    static Constants& getInstance();

private:

    Constants(/* args */) {
        startGoal.header.frame_id = "map";  // Assuming the frame is 'map'
        startGoal.pose.position.x = 0.0903566;    // Set your desired x-coordinate
        startGoal.pose.position.y = 2.36692;    // Set your desired y-coordinate
        startGoal.pose.orientation.z = 0.694793; // Set your desired orientation
        startGoal.pose.orientation.w = 0.694793; // Set your desired orientation

        endGoal.header.frame_id = "map";  // Assuming the frame is 'map'
        endGoal.pose.position.x = -3.00931;    // Set your desired x-coordinate
        endGoal.pose.position.y = 1.00104;    // Set your desired y-coordinate
        endGoal.pose.orientation.z = 1;     // Set your desired orientation
        endGoal.pose.orientation.w = 0.000979705; // Set your desired orientation
    }

    static Constants* instance_;
};
