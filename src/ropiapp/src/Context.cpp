/**
 * @file Context.hpp
 *
 * @brief Header file for the Context class.
 *
 * This file defines the Context class, which manages the state transitions
 * and execution of the pick-and-place application.
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

#include "Context.hpp"

/**
 * @brief Constructor for the Context class.
 */
Context::Context()
    : Node("pick_and_place") {
}

/**
 * @brief Sets the state of the context to the given state.
 *
 * Clears the current state and pushes the new state onto the execution queue.
 *
 * @param _state The new state to set.
 */
void Context::setState(std::shared_ptr<State> _state) {
    runQueue.pop();
    runQueue.push(_state);
    execute();
}

/**
 * @brief Executes the current state in the execution queue.
 */
void Context::execute() {
    RCLCPP_INFO(get_logger(), "Executing next state!");
    std::shared_ptr<State> s = runQueue.front();
    s->Execute();
}

/**
 * @brief Main function for the pick-and-place application.
 *
 * Initializes ROS2, creates a Context node, sets the initial state to the
 * start goal, and enters the main event loop.
 *
 * @param argc The number of command line arguments.
 * @param argv The array of command line arguments.
 * @return int The exit status of the application.
 */
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    std::shared_ptr<Context> node = std::make_shared<Context>();
    std::shared_ptr<GoalManager> startGoal(new GoalManager(*node));
    startGoal->SetPose(Constants::getInstance().startGoal);
    node->setState(startGoal);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
