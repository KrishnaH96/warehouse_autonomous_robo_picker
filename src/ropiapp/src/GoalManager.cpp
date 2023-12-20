#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"  
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "action_msgs/msg/goal_status_array.hpp"


#include "GoalManager.hpp"
#include <functional>
#include <rclcpp/logging.hpp>

GoalManager::GoalManager(Context&  contextPtr): State(contextPtr)
    
{

    goal_client_ = (rclcpp_action::create_client<NavigateToPose>(&contextPtr_, "navigate_to_pose"));
    // Wait for the action server to be available
    while (!goal_client_->wait_for_action_server(std::chrono::seconds(2)))
    {
      RCLCPP_INFO(contextPtr_.get_logger(), "Waiting for the navigate_to_pose action server...");
      rclcpp::sleep_for(std::chrono::seconds(2));
    }
}

 void GoalManager::Execute() {
    RCLCPP_INFO(contextPtr_.get_logger(), "GoalManager Execute!!");
    PublishGoal();
 }

void GoalManager::PublishGoal()
{
    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = pose_;

    // Send the goal with a progress callback
    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    // send_goal_options.feedback_callback = std::bind(&GoalManager::feedback_callback, this, 
    //                         std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback = std::bind(&GoalManager::result_callback, this, 
                                                    std::placeholders::_1);

    RCLCPP_INFO(contextPtr_.get_logger(), "Sending Goal!!");
    auto goal_handle_future_ = goal_client_->async_send_goal(goal_msg, send_goal_options);
    RCLCPP_INFO(contextPtr_.get_logger(), "Goal waiting!!");
}

 void GoalManager::result_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &result)
  {
    switch (result.code) 
    {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(contextPtr_.get_logger(), "Goal was suceeded");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(contextPtr_.get_logger(), "Goal was aborted");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(contextPtr_.get_logger(), "Goal was canceled");
        break;
      default:
        RCLCPP_ERROR(contextPtr_.get_logger(), "Unknown result code");
        break;
    }


    // if (IsStartSate) {
    //     RCLCPP_INFO(contextPtr_.get_logger(), "End Goal Pose was sent!!");
    //     std::shared_ptr<GoalManager> endGoal(new GoalManager(contextPtr_));
    //     endGoal->IsStartSate = false;
    //     endGoal->SetPose(Constants::getInstance().endGoal);
    //     contextPtr_.setState(endGoal);
    // }

   
  }

//  void GoalManager::feedback_callback(rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr, 
//                             const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
//   {
//     // auto distance_feedback_msg = std_msgs::msg::String();
//     // distance_feedback_msgresult_callback.data = "Remaining Distance from Destination: " + std::to_string(feedback->distance_remaining);
//     // RCLCPP_INFO_STREAM(contextPtr_.get_logger(), "Remaining Distance from Destination: " << feedback->distance_remaining);
//   }


    void GoalManager::SetPose(geometry_msgs::msg::PoseStamped pose) {
        pose_ = pose;
    }





// https://robotics.stackexchange.com/questions/98544/ros2-foxy-rclcpp-action-no-match-for-operator