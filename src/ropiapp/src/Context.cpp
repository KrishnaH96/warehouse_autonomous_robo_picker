#include "Context.hpp"

#include "Constants.hpp"
#include "GoalManager.hpp"


Context::Context()
    : Node("pick_and_place") {

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


  std::shared_ptr<GoalManager> startGoal( new GoalManager(*node));
  startGoal->SetPose(Constants::getInstance().startGoal);
  node->setState(startGoal);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}