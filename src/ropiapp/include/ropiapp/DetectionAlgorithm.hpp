/**
 * @file DetectionAlgorithm.hpp
 * @brief Class for defining the detection algorithm.
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
 * @class DetectionAlgorithm
 * @brief A class representing a detection algorithm with specific functionalities.
 */
class DetectionAlgorithm {
public:
    /**
     * @brief Publishes data to a topic related to detected objects.
     *
     * This function simulates the publication of data related to detected objects,
     * such as the gods, to a specific ROS 2 topic.
     *
     * @return true if the publication is successful, false otherwise.
     */
    bool publishGods();

    /**
     * @brief Retrieves the current location information based on the detection algorithm.
     *
     * This function computes and returns the current location information based on the
     * detection algorithm's internal calculations.
     *
     * @return Current location information, typically in a suitable data structure.
     */
    auto getLocation();
};