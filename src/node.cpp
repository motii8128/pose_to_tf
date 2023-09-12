// Copyright 2023 Hakoroboken
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
#include <chrono>
#include <vector>
#include <cstdlib>
#include <random>

#include "pose_to_tf/node.hpp"

namespace pose_to_tf
{
    PoseTF::PoseTF(const rclcpp::NodeOptions &node_option)
        : rclcpp::Node("pose_to_tf", node_option)
    {
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/pose_stamped", 0,
            std::bind(&PoseTF::sub_callback, this, std::placeholders::_1));
    }

    void PoseTF::sub_callback(const std::shared_ptr<geometry_msgs::msg::PoseStamped> msg)
    {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "map";
        t.child_frame_id = "base_link";

        t.transform.translation.x = msg->pose.position.x;
        t.transform.translation.y = msg->pose.position.y;
        t.transform.translation.z = msg->pose.position.z;

        t.transform.rotation.x = msg->pose.orientation.x;
        t.transform.rotation.y = msg->pose.orientation.y;
        t.transform.rotation.z = msg->pose.orientation.z;
        t.transform.rotation.w = msg->pose.orientation.w;

        tf_broadcaster_->sendTransform(t);
    }
}


#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pose_to_tf::PoseTF)