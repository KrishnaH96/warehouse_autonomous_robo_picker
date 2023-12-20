/**
 * @file RobotAligner.hpp
 * @brief A class representing a robot aligner for orientation and movement.
 * @author Krishna Rajesh Hundekari
 * @date 2023-12-19
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

#include "State.hpp"

/**
 * @class PickObject
 * @brief A class representing the pick action state for the robot picker.
 */
class PickObject: public State {
public:
    /**
     * @brief Moves the robots fork lift position to pick locaiton.
     *
     * @return True if the movement alignment is successful, false otherwise.
     */
    bool moveUpForkLift();

    /**
     * @brief State entry point
     * 
     */
    void execute();
};