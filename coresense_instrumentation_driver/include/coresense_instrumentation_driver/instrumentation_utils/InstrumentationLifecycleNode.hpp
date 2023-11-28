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

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace coresense_instrumentation_driver
{

class InstrumentationLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  InstrumentationLifecycleNode(
    const std::string & node_name,
    const std::string & ns = "",
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~LifecycleNode();
};

} // namespace coresense_instrumentation_driver