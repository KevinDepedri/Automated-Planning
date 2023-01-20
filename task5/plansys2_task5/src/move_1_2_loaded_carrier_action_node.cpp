// Copyright 2019 Intelligent Robotics Lab
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

#include <memory>
#include <algorithm>

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

class FillBoxWithSupplyAction : public plansys2::ActionExecutorClient
{
public:
  // Each 250ms the method do_work() of the class MoveAction() is called
  FillBoxWithSupplyAction() : plansys2::ActionExecutorClient("move_1_2_loaded_carrier", 200ms){
    progress_ = 0.0;
  }

private:
  float duration = 6000;
  float calls = duration/200;
  float increment = 1/calls;
  void do_work()
  {
    if (progress_ < 1.0) {
      progress_ += increment;
      // Inform on the terminal the current state of the process through send_feedback
      send_feedback(progress_, "Moving 1-2 loaded carrier in progress");
    } else {
      // Notify that the action is finished using finish. Then moves the node into inactive state
      finish(true, 1.0, "Moving 1-2 loaded carrier completed");

      progress_ = 0.0;
      std::cout << std::endl;
    }

    // Give additional output, not needed in assignment
    std::cout << "\r\e[K" << std::flush;
    std::cout << "Moving 1-2 loaded carrier ... [" << std::min(100.0, progress_ * 100.0) << "%]  " <<
    std::flush;
  }

  float progress_;
};

int main(int argc, char ** argv)
{
  // Initializa the ROS infrastructure
  rclcpp::init(argc, argv);
  // Define the action node
  auto node = std::make_shared<FillBoxWithSupplyAction>();

  // Define the name of the action node
  node->set_parameter(rclcpp::Parameter("action_name", "move_1_2_loaded_carrier"));
  // Configure the node and let it inactive but ready to be called
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  // Look for an activation of the node. When the planner will need this action to
  // achieve its goal it will call and activate the current node
  rclcpp::spin(node->get_node_base_interface());

  // ??
  rclcpp::shutdown();

  return 0;
}
