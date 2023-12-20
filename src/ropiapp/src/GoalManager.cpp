/**
 * @file GoalManager.cpp
 *
 * @brief Manages the goal aspects to ensure the autonomy of the robot.
 *
 * @author Tej, Krishna, Abraruddin
 *
 * @license Apache License Version 2.0, January 2004
 *
 * Copyright 2023 Tej, Krishna, Abraruddin
 * 
 * @details
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements. See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership. The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied. See the License for the
 * specific language governing permissions and limitations
 * under the License.
 * 
 *  
 */
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "action_msgs/msg/goal_status_array.hpp"

#include "GoalManager.hpp"
#include <functional>
#include <rclcpp/logging.hpp>

/**
 * @brief Constructor for GoalManager class.
 *
 * Initializes the GoalManager with the given context.
 *
 * @param contextPtr The context object used to maintain the state.
 */
GoalManager::GoalManager(Context& contextPtr)
    : State(contextPtr) {
    goal_client_ = rclcpp_action::create_client
      <NavigateToPose>(&contextPtr_, "navigate_to_pose");

    // Wait for the action server to be available
    while (!goal_client_->wait_for_action_server(std::chrono::seconds(2))) {
        RCLCPP_INFO(contextPtr_.get_logger(),
          "Waiting for the navigate_to_pose action server...");
        rclcpp::sleep_for(std::chrono::seconds(2));
    }
}

/**
 * @brief Executes the GoalManager state.
 */
void GoalManager::Execute() {
    RCLCPP_INFO(contextPtr_.get_logger(), "GoalManager Execute!!");
    PublishGoal();
}

/**
 * @brief Publishes the goal pose to the navigate_to_pose action server.
 */
void GoalManager::PublishGoal() {
    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = pose_;

    // Send the goal with a result callback
    auto send_goal_options = rclcpp_action::Client
      <NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback = std::bind(
        &GoalManager::result_callback, this, std::placeholders::_1);

    RCLCPP_INFO(contextPtr_.get_logger(), "Sending Goal!!");
    goal_client_->async_send_goal(goal_msg, send_goal_options);
    RCLCPP_INFO(contextPtr_.get_logger(), "Goal waiting!!");
}

/**
 * @brief Callback function for the result of the navigate_to_pose action.
 *
 * Handles different result codes and logs appropriate messages.
 *
 * @param result The result of the action.
 */
void GoalManager::result_callback(const rclcpp_action::ClientGoalHandle
  <nav2_msgs::action::NavigateToPose>::WrappedResult& result) {
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(contextPtr_.get_logger(), "Goal was succeeded");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(contextPtr_.get_logger(), "Goal was aborted");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(contextPtr_.get_logger(), "Goal was canceled");
            break;
        default:
            RCLCPP_ERROR(contextPtr_.get_logger(), "Unknown result code");
            break;
    }

    // Example of transitioning to a new state after goal completion
    // if (IsStartSate) {
    //     RCLCPP_INFO(contextPtr_.get_logger(), "End Goal Pose was sent!!");
    //     std::shared_ptr<GoalManager> endGoal(new GoalManager(contextPtr_));
    //     endGoal->IsStartSate = false;
    //     endGoal->SetPose(Constants::getInstance().endGoal);
    //     contextPtr_.setState(endGoal);
    // }
}

/**
 * @brief Sets the goal pose for the navigation action.
 *
 * @param pose The goal pose to set.
 */
void GoalManager::SetPose(geometry_msgs::msg::PoseStamped pose) {
    pose_ = pose;
}

// https://robotics.stackexchange.com/questions/98544/ros2-foxy-rclcpp-action-no-match-for-operator
