#include "Context.hpp"

#include "Constants.hpp"
#include "GoalManager.hpp"


Context::Context()
    : Node("pick_and_place") {

    RCLCPP_INFO(get_logger(), "Task started..");

    std::shared_ptr<GoalManager> startGoal( new GoalManager(*this));
    startGoal->SetPose(Constants::getInstance().startGoal);

    runQueue.push(startGoal);

    execute();
}

void Context::setState(std::shared_ptr<State> _state) {
    runQueue.pop();
    runQueue.push(_state);
    execute();
}

void Context::execute() {
        RCLCPP_INFO(get_logger(), "Executing next state!");
        std::shared_ptr<State> s = runQueue.front();
        s->Execute();
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<Context> node =   std::make_shared<Context>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}