// Copyright 2023 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "coresense_instrumentation_driver/InstrumentationLifecycleNode.hpp"


namespace coresense_instrumentation_driver
{

template class coresense_instrumentation_driver::InstrumentationLifecycleNode<sensor_msgs::msg::LaserScan>;
template class coresense_instrumentation_driver::InstrumentationLifecycleNode<std_msgs::msg::String>;

template<typename TopicT>
InstrumentationLifecycleNode<TopicT>::InstrumentationLifecycleNode(
  const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("lifecycle_node", "", options)
{
  declare_parameter("topic", std::string(""));
  declare_parameter("topic_type", std::string(""));

  get_parameter("topic", topic_);
  get_parameter("topic_type", topic_type_);

  RCLCPP_INFO(get_logger(), "Creating InstrumentationGeneric");
}

template<typename TopicT>
InstrumentationLifecycleNode<TopicT>::~InstrumentationLifecycleNode()
{
  RCLCPP_DEBUG(get_logger(), "Destroying InstrumentationGeneric");
}

template<typename TopicT>
std::string InstrumentationLifecycleNode<TopicT>::get_topic()
{
  return topic_;
}

template<typename TopicT>
std::string InstrumentationLifecycleNode<TopicT>::get_topic_type()
{
  return topic_type_;
}

template<typename TopicT>
typename rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
InstrumentationLifecycleNode<TopicT>::on_configure(const rclcpp_lifecycle::State &)
{
  sub_ = this->create_subscription<TopicT>(
    topic_, 10,
    [this](const typename TopicT::SharedPtr msg) {
      for (auto & pub : publishers_) {
        if (pub.second->get_subscription_count() > 0 && pub.second->is_activated()) {
          pub.second->publish(std::make_unique<TopicT>(*msg));
        }
      }
    });

  if (topic_[0] == '/') {
    topic_ = topic_.substr(1);
  }

  auto pub = this->create_publisher<TopicT>("/coresense/" + topic_, 10);
  publishers_.insert({topic_, pub});

  create_publisher_service_ =
    this->create_service<coresense_instrumentation_interfaces::srv::CreatePublisher>(
    "/coresense" + topic_ + "/create_publisher",
    std::bind(
      &InstrumentationLifecycleNode<TopicT>::handleCreatePublisherRequest, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  delete_publisher_service_ =
    this->create_service<coresense_instrumentation_interfaces::srv::DeletePublisher>(
    "/coresense" + topic_ + "/delete_publisher",
    std::bind(
      &InstrumentationLifecycleNode<TopicT>::handleDeletePublisherRequest, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

template<typename TopicT>
typename rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
InstrumentationLifecycleNode<TopicT>::on_activate(const rclcpp_lifecycle::State &)
{
  for (auto & pub : publishers_) {
    pub.second->on_activate();
  }

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

template<typename TopicT>
typename rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
InstrumentationLifecycleNode<TopicT>::on_deactivate(const rclcpp_lifecycle::State &)
{
  for (auto & pub : publishers_) {
    pub.second->on_deactivate();
  }

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

template<typename TopicT>
typename rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
InstrumentationLifecycleNode<TopicT>::on_cleanup(const rclcpp_lifecycle::State &)
{
  for (auto & pub : publishers_) {
    pub.second.reset();
  }

  sub_.reset();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

template<typename TopicT>
typename rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
InstrumentationLifecycleNode<TopicT>::on_shutdown(const rclcpp_lifecycle::State &)
{
  for (auto & pub : publishers_) {
    pub.second.reset();
  }

  sub_.reset();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

template<typename TopicT>
void InstrumentationLifecycleNode<TopicT>::handleCreatePublisherRequest(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<coresense_instrumentation_interfaces::srv::CreatePublisher::Request> request,
  const std::shared_ptr<coresense_instrumentation_interfaces::srv::CreatePublisher::Response> response)
{
  (void)request_header;

  std::string new_topic = request->topic_name;

  if (new_topic[0] == '/') {
    new_topic = new_topic.substr(1);
  }

  for (auto & pub : publishers_) {
    if (pub.first == new_topic) {
      response->success = false;
      return;
    }
  }

  auto new_pub = this->create_publisher<TopicT>("/coresense/" + new_topic, 10);

  if (this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    new_pub->on_activate();
  }

  publishers_.insert({new_topic, new_pub});
  response->success = true;
}

template<typename TopicT>
void InstrumentationLifecycleNode<TopicT>::handleDeletePublisherRequest(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<coresense_instrumentation_interfaces::srv::DeletePublisher::Request> request,
  const std::shared_ptr<coresense_instrumentation_interfaces::srv::DeletePublisher::Response> response)
{
  (void)request_header;

  std::string remove_topic = request->topic_name;

  if (remove_topic[0] == '/') {
    remove_topic = remove_topic.substr(1);
  }

  for (auto & pub : publishers_) {
    if (pub.first == remove_topic) {
      pub.second.reset();
    }
  }

  publishers_.erase(remove_topic);
  response->success = true;
}

} // namespace coresense_instrumentation_driver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(
  coresense_instrumentation_driver::InstrumentationLifecycleNode<sensor_msgs::msg::LaserScan>)
RCLCPP_COMPONENTS_REGISTER_NODE(
  coresense_instrumentation_driver::InstrumentationLifecycleNode<std_msgs::msg::String>)
