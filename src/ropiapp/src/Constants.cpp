/**
 * @file Constants.hpp
 *
 * @brief Header file for Constants class.
 *
 * This file defines the Constants class, which is a singleton class
 * providing access to application-wide constants.
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

#include "Constants.hpp"

Constants* Constants::instance_;

/**
 * @brief Gets the singleton instance of the Constants class.
 *
 * If the instance does not exist, it creates a new instance.
 *
 * @return Constants& The singleton instance of the Constants class.
 */
Constants& Constants::getInstance() {
    if (Constants::instance_ == nullptr) {
        Constants::instance_ = new Constants();
    }
    return *Constants::instance_;
}
