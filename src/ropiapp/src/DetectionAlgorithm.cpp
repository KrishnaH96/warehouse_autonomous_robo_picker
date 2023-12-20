/**
 * @file DetectionAlgorithm.cpp
 *
 * @brief The context object used to detect aruco marker.
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

#include "DetectionAlgorithm.hpp"
#include "PickObject.hpp"
#include "RobotAligner.hpp"

/**
 * @brief Constructor for DetectionAlgorithm class.
 *
 * @param contextPtr The context object used to maintain the state.
 */
DetectionAlgorithm::DetectionAlgorithm(Context& contextPtr)
    : State(contextPtr) {
}

/**
 * @brief Detects goods in the environment.
 *
 * @return Returns true if goods are detected, false otherwise.
 */
bool DetectionAlgorithm::detectGoods() {
    // To Do
}

/**
 * @brief Executes the detection algorithm.
 *
 * If goods are not detected, it looks for Aruco codes and rotates the robot
 * in place. If Aruco code is detected, it initiates the object-picking process.
 */
void DetectionAlgorithm::execute() {
    if (!detectGoods()) {
        /**
         * @brief Look for Aruco codes and rotate the robot if no code is detected.
         */
        std::shared_ptr<RobotAligner>
            rotateInplace(new RobotAligner(contextPtr_));
        contextPtr_.setState(rotateInplace);
    } else {
        /**
         * @brief If Aruco code is detected, pick the object.
         *
         * @return std::shared_ptr<PickObject>
         */
        std::shared_ptr<PickObject> pickObject(new PickObject(contextPtr_));
        contextPtr_.setState(pickObject);
    }
}
