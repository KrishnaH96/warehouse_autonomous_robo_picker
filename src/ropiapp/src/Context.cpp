#include "Context.hpp"

#include "Constants.hpp"
#include "GoalManager.hpp"


Context::Context()
    : Node("pick_and_place") {


    RCLCPP_INFO(get_logger(), "Execution started..");
    std::shared_ptr<GoalManager> startGoal( new GoalManager(*this));
    startGoal->SetPose(Constants::getInstance().startGoal);

    RCLCPP_INFO(get_logger(), "Pushed First state!");
    runQueue.push(startGoal);

    execute();
}

void Context::setState(std::shared_ptr<State> _state) {
    RCLCPP_INFO(get_logger(), "Removing top state!");
    runQueue.pop();

    runQueue.push(_state);

    execute();
}

void Context::execute() {

        RCLCPP_INFO(get_logger(), "Fetched top state!");
        std::shared_ptr<State> s = runQueue.front();

        RCLCPP_INFO(get_logger(), "Executing top state!");
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