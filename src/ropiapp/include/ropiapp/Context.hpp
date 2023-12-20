#pragma once

#include <memory>
#include <queue>

#include "rclcpp/rclcpp.hpp"

class State;

class Context :  public rclcpp::Node
{
    private:
        std::queue<std::shared_ptr<State>> runQueue;

    public:
        Context();
        void setState(std::shared_ptr<State> _state);
        void execute();
};


