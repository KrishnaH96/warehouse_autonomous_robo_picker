/**
 * @file PickPlaceTask.hpp
 * @brief A class representing a task involving picking and placing objects.
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

/**
 * @class PickPlaceTask
 * @brief A class representing a task involving picking and placing objects.
 */
class PickPlaceTask {
public:
    /**
     * @brief Runs the pick and place task.
     *
     * This function simulates the execution of a pick and place task, where an object is
     * picked from a starting location and placed at an end goal location.
     *
     * @return True if the task is executed successfully, false otherwise.
     */
    bool runTask();

private:
    /**
     * @brief The starting location for the pick and place task.
     */
    double startGoal;

    /**
     * @brief The goal location where the object should be placed.
     */
    double endGoal;
};