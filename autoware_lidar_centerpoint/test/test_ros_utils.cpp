// Copyright 2024 TIER IV, Inc.
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

#include "autoware/lidar_centerpoint/ros_utils.hpp"

#include <gtest/gtest.h>

#include <perception_msgs_utils/object_access.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <string>
#include <vector>

TEST(TestSuite, box3DToDetectedObject)
{
  std::vector<std::string> class_names = {"CAR",     "TRUCK",     "BUS",       "TRAILER",
                                          "BICYCLE", "MOTORBIKE", "PEDESTRIAN"};

  // Test case 1: Test with valid label, has_twist=true, has_variance=true
  {
    autoware::lidar_centerpoint::Box3D box3d;
    box3d.score = 0.8f;
    box3d.label = 0;  // CAR
    box3d.x = 1.0;
    box3d.y = 2.0;
    box3d.z = 3.0;
    box3d.yaw = 0.5;
    box3d.length = 4.0;
    box3d.width = 2.0;
    box3d.height = 1.5;
    box3d.vel_x = 1.0;
    box3d.vel_y = 0.5;
    box3d.x_variance = 0.1;
    box3d.y_variance = 0.2;
    box3d.z_variance = 0.3;
    box3d.yaw_variance = 0.4;
    box3d.vel_x_variance = 0.5;
    box3d.vel_y_variance = 0.6;

    autoware_perception_msgs::msg::DetectedObject obj;
    autoware::lidar_centerpoint::box3DToDetectedObject(box3d, class_names, true, true, obj);

    EXPECT_FLOAT_EQ(obj.existence_probability, 0.8f);
    EXPECT_EQ(
      obj.classification[0].label, autoware_perception_msgs::msg::ObjectClassification::CAR);
    EXPECT_FLOAT_EQ(obj.kinematics.pose_with_covariance.pose.position.x, 1.0);
    EXPECT_FLOAT_EQ(obj.kinematics.pose_with_covariance.pose.position.y, 2.0);
    EXPECT_FLOAT_EQ(obj.kinematics.pose_with_covariance.pose.position.z, 3.0);
    EXPECT_FLOAT_EQ(obj.shape.dimensions.x, 4.0);
    EXPECT_FLOAT_EQ(obj.shape.dimensions.y, 2.0);
    EXPECT_FLOAT_EQ(obj.shape.dimensions.z, 1.5);
    EXPECT_TRUE(obj.kinematics.has_position_covariance);
    EXPECT_TRUE(obj.kinematics.has_twist);
    EXPECT_TRUE(obj.kinematics.has_twist_covariance);
  }

  // Test case 2: Test with invalid label, has_twist=false, has_variance=false
  {
    autoware::lidar_centerpoint::Box3D box3d;
    box3d.score = 0.5f;
    box3d.label = 10;  // Invalid

    autoware_perception_msgs::msg::DetectedObject obj;
    autoware::lidar_centerpoint::box3DToDetectedObject(box3d, class_names, false, false, obj);

    EXPECT_FLOAT_EQ(obj.existence_probability, 0.5f);
    EXPECT_EQ(
      obj.classification[0].label, autoware_perception_msgs::msg::ObjectClassification::UNKNOWN);
    EXPECT_FALSE(obj.kinematics.has_position_covariance);
    EXPECT_FALSE(obj.kinematics.has_twist);
    EXPECT_FALSE(obj.kinematics.has_twist_covariance);
  }
}

TEST(TestSuite, getSemanticType)
{
  EXPECT_EQ(
    autoware::lidar_centerpoint::getSemanticType("CAR"),
    autoware_perception_msgs::msg::ObjectClassification::CAR);
  EXPECT_EQ(
    autoware::lidar_centerpoint::getSemanticType("TRUCK"),
    autoware_perception_msgs::msg::ObjectClassification::TRUCK);
  EXPECT_EQ(
    autoware::lidar_centerpoint::getSemanticType("BUS"),
    autoware_perception_msgs::msg::ObjectClassification::BUS);
  EXPECT_EQ(
    autoware::lidar_centerpoint::getSemanticType("TRAILER"),
    autoware_perception_msgs::msg::ObjectClassification::TRAILER);
  EXPECT_EQ(
    autoware::lidar_centerpoint::getSemanticType("BICYCLE"),
    autoware_perception_msgs::msg::ObjectClassification::BICYCLE);
  EXPECT_EQ(
    autoware::lidar_centerpoint::getSemanticType("MOTORBIKE"),
    autoware_perception_msgs::msg::ObjectClassification::MOTORCYCLE);
  EXPECT_EQ(
    autoware::lidar_centerpoint::getSemanticType("PEDESTRIAN"),
    autoware_perception_msgs::msg::ObjectClassification::PEDESTRIAN);
  EXPECT_EQ(
    autoware::lidar_centerpoint::getSemanticType("UNKNOWN"),
    autoware_perception_msgs::msg::ObjectClassification::UNKNOWN);
}

TEST(TestSuite, convertPoseCovarianceMatrix)
{
  autoware::lidar_centerpoint::Box3D box3d;
  box3d.x_variance = 0.1;
  box3d.y_variance = 0.2;
  box3d.z_variance = 0.3;
  box3d.yaw_variance = 0.4;

  std::array<double, 36> pose_covariance =
    autoware::lidar_centerpoint::convertPoseCovarianceMatrix(box3d);

  EXPECT_FLOAT_EQ(pose_covariance[0], 0.1);
  EXPECT_FLOAT_EQ(pose_covariance[7], 0.2);
  EXPECT_FLOAT_EQ(pose_covariance[14], 0.3);
  EXPECT_FLOAT_EQ(pose_covariance[35], 0.4);
}

TEST(TestSuite, convertTwistCovarianceMatrix)
{
  autoware::lidar_centerpoint::Box3D box3d;
  box3d.vel_x_variance = 0.5;
  box3d.vel_y_variance = 0.2;
  float yaw = 0;

  std::array<double, 36> twist_covariance =
    autoware::lidar_centerpoint::convertTwistCovarianceMatrix(box3d, yaw);

  EXPECT_FLOAT_EQ(twist_covariance[0], 0.5);
  EXPECT_FLOAT_EQ(twist_covariance[7], 0.2);
}

TEST(TestSuite, detectedObjectsToObjectList)
{
  autoware_perception_msgs::msg::DetectedObjects detected_objects_msg;
  detected_objects_msg.header.frame_id = "base_link";
  detected_objects_msg.header.stamp.sec = 123;
  detected_objects_msg.header.stamp.nanosec = 456;

  autoware_perception_msgs::msg::DetectedObject detected_object;
  detected_object.existence_probability = 0.8;

  autoware_perception_msgs::msg::ObjectClassification classification;
  classification.label = autoware_perception_msgs::msg::ObjectClassification::CAR;
  classification.probability = 0.9F;
  detected_object.classification.push_back(classification);

  detected_object.kinematics.pose_with_covariance.pose.position.x = 1.0;
  detected_object.kinematics.pose_with_covariance.pose.position.y = 2.0;
  detected_object.kinematics.pose_with_covariance.pose.position.z = 3.0;
  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, 0.25);
  detected_object.kinematics.pose_with_covariance.pose.orientation = tf2::toMsg(orientation);
  detected_object.kinematics.has_position_covariance = true;
  detected_object.kinematics.pose_with_covariance.covariance[0] = 0.1;
  detected_object.kinematics.pose_with_covariance.covariance[7] = 0.2;
  detected_object.kinematics.pose_with_covariance.covariance[14] = 0.3;
  detected_object.kinematics.pose_with_covariance.covariance[35] = 0.4;

  detected_object.shape.dimensions.x = 4.0;
  detected_object.shape.dimensions.y = 2.0;
  detected_object.shape.dimensions.z = 1.5;

  detected_object.kinematics.has_twist = true;
  detected_object.kinematics.has_twist_covariance = true;
  detected_object.kinematics.twist_with_covariance.twist.linear.x = 3.0;
  detected_object.kinematics.twist_with_covariance.twist.linear.y = 1.0;
  detected_object.kinematics.twist_with_covariance.covariance[0] = 0.5;
  detected_object.kinematics.twist_with_covariance.covariance[1] = 0.1;
  detected_object.kinematics.twist_with_covariance.covariance[6] = 0.1;
  detected_object.kinematics.twist_with_covariance.covariance[7] = 0.2;

  detected_objects_msg.objects.push_back(detected_object);

  const auto object_list =
    autoware::lidar_centerpoint::detectedObjectsToObjectList(detected_objects_msg);

  ASSERT_EQ(object_list.objects.size(), 1U);
  EXPECT_EQ(object_list.header.frame_id, detected_objects_msg.header.frame_id);
  EXPECT_EQ(object_list.header.stamp, detected_objects_msg.header.stamp);

  const auto & output_object = object_list.objects.front();
  EXPECT_EQ(output_object.id, 1U);
  EXPECT_DOUBLE_EQ(output_object.existence_probability, detected_object.existence_probability);
  EXPECT_EQ(output_object.state.header.frame_id, detected_objects_msg.header.frame_id);
  EXPECT_EQ(output_object.state.header.stamp, detected_objects_msg.header.stamp);
  ASSERT_EQ(output_object.state.classifications.size(), 1U);
  EXPECT_EQ(
    output_object.state.classifications.front().type,
    perception_msgs::msg::ObjectClassification::CAR);
  EXPECT_DOUBLE_EQ(
    output_object.state.classifications.front().probability,
    static_cast<double>(classification.probability));

  const auto pose_with_covariance = perception_msgs::object_access::getPoseWithCovariance(output_object);
  EXPECT_DOUBLE_EQ(pose_with_covariance.pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(pose_with_covariance.pose.position.y, 2.0);
  EXPECT_DOUBLE_EQ(pose_with_covariance.pose.position.z, 3.0);
  EXPECT_NEAR(perception_msgs::object_access::getYaw(output_object), 0.25, 1e-6);
  EXPECT_DOUBLE_EQ(pose_with_covariance.covariance[0], 0.1);
  EXPECT_DOUBLE_EQ(pose_with_covariance.covariance[7], 0.2);
  EXPECT_DOUBLE_EQ(pose_with_covariance.covariance[14], 0.3);
  EXPECT_DOUBLE_EQ(pose_with_covariance.covariance[35], 0.4);

  EXPECT_DOUBLE_EQ(perception_msgs::object_access::getLength(output_object), 4.0);
  EXPECT_DOUBLE_EQ(perception_msgs::object_access::getWidth(output_object), 2.0);
  EXPECT_DOUBLE_EQ(perception_msgs::object_access::getHeight(output_object), 1.5);

  const auto velocity = perception_msgs::object_access::getVelocity(output_object);
  EXPECT_DOUBLE_EQ(velocity.x, 3.0);
  EXPECT_DOUBLE_EQ(velocity.y, 1.0);

  constexpr auto state_size = perception_msgs::msg::ISCACTR::CONTINUOUS_STATE_SIZE;
  constexpr auto vel_lon = perception_msgs::msg::ISCACTR::VEL_LON;
  constexpr auto vel_lat = perception_msgs::msg::ISCACTR::VEL_LAT;
  EXPECT_DOUBLE_EQ(
    output_object.state.continuous_state_covariance[state_size * vel_lon + vel_lon], 0.5);
  EXPECT_DOUBLE_EQ(
    output_object.state.continuous_state_covariance[state_size * vel_lon + vel_lat], 0.1);
  EXPECT_DOUBLE_EQ(
    output_object.state.continuous_state_covariance[state_size * vel_lat + vel_lon], 0.1);
  EXPECT_DOUBLE_EQ(
    output_object.state.continuous_state_covariance[state_size * vel_lat + vel_lat], 0.2);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
