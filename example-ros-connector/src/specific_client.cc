/**
 * Copyright 2016-2025 California Institute of Technology
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <future>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "example_msgs/action/example_action.hpp"
#include "example_ros_connector/specific_client.h"

namespace trace {
namespace example_ros_connector{

SpecificClient::SpecificClient()
: Node("default_specific_client") {
    this->example_ac_ = rclcpp_action::create_client<example_msgs::action::ExampleAction>(this, "example_action_server");

    // Start a separate thread to spin the node after initializing 
    // and before handling action goals to manage callbacks while RunTask waits.
    std::thread([this](){
        rclcpp::spin(this->get_node_base_interface());
    }).detach();  // Detach the thread to let it run independently
}

SpecificClient::SpecificClient(const std::string &node_name)
: Node(node_name) {
    // Additional initialization can be done here if necessary
}

void SpecificClient::Connect() {
    if (!example_ac_->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Action server is available.");
}

std::future<trace::Outcome> SpecificClient::SendCommand(const std::string &service_task_uuid,
                                                          const PropertyMap &properties) {
    return std::async(std::launch::async, &SpecificClient::RunTask, this, service_task_uuid, properties);
}

void SpecificClient::AbortCommand(const std::string &service_task_uuid) {
    // Implement action cancellation logic here
    return;
}

trace::Outcome SpecificClient::RunTask(const std::string &service_task_uuid,
                                       const PropertyMap &properties) {

    auto goal_msg = example_msgs::action::ExampleAction::Goal();
    goal_msg.server_input = 10;

    auto send_goal_options = rclcpp_action::Client<example_msgs::action::ExampleAction>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&SpecificClient::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
        std::bind(&SpecificClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
        std::bind(&SpecificClient::result_callback, this, std::placeholders::_1);

    result_promise_ = std::promise<float>();  // Reset the promise
    auto result_future = result_promise_.get_future();
    example_ac_->async_send_goal(goal_msg, send_goal_options);

    float action_result;
    try {
        action_result = result_future.get(); 
    } catch (const std::future_error& e) {
        RCLCPP_ERROR(this->get_logger(), "Future error: %s", e.what());
        return trace::Outcome(StatusCode::ERROR, "Failed to get future result");
    }

    RCLCPP_ERROR(this->get_logger(), "ABOUT TO PRINT PROPERTY KEY AND VALUE");
    for (auto const& pair : properties) {
        RCLCPP_ERROR(this->get_logger(), "Property key: %s, value: %s", pair.first.c_str(), pair.second.c_str());
    }
 
    int number_of_rovers = std::stoi(properties.at("number_of_active_mars_rovers"));
    float average = (action_result + number_of_rovers) / 2.0;
    // RCLCPP_ERROR(this->get_logger(), "Previous output: %f", prev_output);
    
    RCLCPP_INFO(this->get_logger(), "Average Determined to be: %f", average);

    trace::Outcome ok(StatusCode::OK, "Connector activity completed successfully");
    ok.add_result("average", std::to_string(average));

    return ok;
}


void SpecificClient::goal_response_callback(const rclcpp_action::ClientGoalHandle<example_msgs::action::ExampleAction>::SharedPtr goal_handle) {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

void SpecificClient::feedback_callback(const rclcpp_action::ClientGoalHandle<example_msgs::action::ExampleAction>::SharedPtr,
                                       const std::shared_ptr<const example_msgs::action::ExampleAction::Feedback> feedback) {
    std::stringstream progress_stream;
    for (int progress : feedback->server_progress) {
        progress_stream << progress << " ";
    }
    RCLCPP_INFO(this->get_logger(), "Received feedback: %s", progress_stream.str().c_str());
}

void SpecificClient::result_callback(const rclcpp_action::ClientGoalHandle<example_msgs::action::ExampleAction>::WrappedResult & result) {
    RCLCPP_INFO(this->get_logger(), "Result callback triggered.");
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        float result_value = result.result->server_output;
        result_promise_.set_value(result_value);
        RCLCPP_INFO(this->get_logger(), "Goal succeeded, result: %f", result_value);
    } else {
        result_promise_.set_value(0.0); // Handle failures here
        RCLCPP_ERROR(this->get_logger(), "Goal failed or was cancelled. Result code: %d", result.code);
    }
}

} //example_ros_connector
} //trace