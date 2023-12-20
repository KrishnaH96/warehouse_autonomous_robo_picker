/**
 * @file GoalManager.hpp
 * @brief A class representing a goal management system
 *
 * @author Krishna Rajesh Hundekari
 * @date 2023-12-12
 * @copyright 2023 Krishna Rajesh Hundekari
 * Apache License Version 2.0, January 2004
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements. See the NOTICE file distributed with this
 * work for additional information regarding copyright ownership. The ASF
 * licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License. You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied. See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#pragma once

#include <iostream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"  
#include "std_msgs/msg/string.hpp"  
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "action_msgs/msg/goal_status_array.hpp"

#include "State.hpp"
#include "Constants.hpp"

/**
 * @class GoalManager
 * @brief A class representing a goal management system.
 */
class GoalManager: public State {
    public:
        bool IsStartSate = true;
        using NavigateToPose = nav2_msgs::action::NavigateToPose;
        using GoalStatusArray = action_msgs::msg::GoalStatusArray;
        
        /**
         * @brief Construct a new Goal Manager object
         * 
         */
        GoalManager(Context&  contextPtr);

        /**
         * @brief Execute method for State
         * 
         */
        void Execute();

        /**
         * @brief Set the Pose object
         * 
         * @param pose 
         */
        void SetPose(geometry_msgs::msg::PoseStamped pose);

        /**
         * @brief Publishes information related to a goal.
         *
         * This function simulates the publication of information related to a goal,
         * such as its status, progress, or any relevant details.
         *
         * @return The result of the goal publication, typically indicating success or failure.
         */
        void PublishGoal();

        /**
         * @brief Tracks the progress of a goal.
         *
         * This function tracks the progress of a goal, possibly by monitoring relevant parameters
         * or external conditions associated with the goal.
         *
         * @return True if the goal is being successfully tracked, false otherwise.
         */
        // void feedback_callback(rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr, 
        //                 const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback);

        /**
         * @brief Gets the result for the goal
         * 
         * Callback funciton for the goal reach result
         */
        void result_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &result);


    private:
        geometry_msgs::msg::PoseStamped pose_;
        rclcpp_action::Client<NavigateToPose>::SharedPtr goal_client_ = nullptr;
};