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

#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "example_msgs/action/example_action.hpp" 
#include "service_task_interface.h" 
#include <future>
#include <string>
#include <unordered_map>

namespace trace {
namespace example_ros_connector {

class SpecificClient : public ServiceTaskInterface, public rclcpp::Node {
public:
    // Constructor to initialize the action client
    SpecificClient();

    // Constructor with node name parameter
    explicit SpecificClient(const std::string &node_name);

    // Override virtual functions from the ServiceTaskInterface
    void Connect() override;
    std::future<trace::Outcome> SendCommand(const std::string &service_task_uuid,
                                            const PropertyMap &properties) override;
    void AbortCommand(const std::string &service_task_uuid) override;

    // Destructor
    virtual ~SpecificClient() {}

private:
    // Action client member variable
    rclcpp_action::Client<example_msgs::action::ExampleAction>::SharedPtr example_ac_;

    std::promise<float> result_promise_;

    // Asynchronous goal handling methods
    void goal_response_callback(const rclcpp_action::ClientGoalHandle<example_msgs::action::ExampleAction>::SharedPtr goal_handle);
    void feedback_callback(const rclcpp_action::ClientGoalHandle<example_msgs::action::ExampleAction>::SharedPtr goal_handle,
                           const std::shared_ptr<const example_msgs::action::ExampleAction::Feedback> feedback);
    void result_callback(const rclcpp_action::ClientGoalHandle<example_msgs::action::ExampleAction>::WrappedResult & result);

    // Function that might be used internally by SendCommand
    trace::Outcome RunTask(const std::string &service_task_uuid, const PropertyMap &properties);
};

}  // namespace example_ros_connector
}  // namespace trace
