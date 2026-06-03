// Copyright 2022 TIER IV, Inc.
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

#include <autoware/object_recognition_utils/object_recognition_utils.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/math/constants.hpp>
#include <perception_msgs_utils/object_access.hpp>

#include <string>
#include <vector>

namespace autoware::lidar_centerpoint
{

using Label = autoware_perception_msgs::msg::ObjectClassification;

namespace
{

uint8_t toPerceptionClassificationType(const uint8_t label)
{
  using PerceptionLabel = perception_msgs::msg::ObjectClassification;

  switch (label) {
    case Label::CAR:
      return PerceptionLabel::CAR;
    case Label::TRUCK:
      return PerceptionLabel::TRUCK;
    case Label::BUS:
      return PerceptionLabel::BUS;
    case Label::TRAILER:
      return PerceptionLabel::TRAILER;
    case Label::MOTORCYCLE:
      return PerceptionLabel::MOTORCYCLE;
    case Label::BICYCLE:
      return PerceptionLabel::BICYCLE;
    case Label::PEDESTRIAN:
      return PerceptionLabel::PEDESTRIAN;
    case Label::ANIMAL:
      return PerceptionLabel::ANIMAL;
    case Label::HAZARD:
    case Label::OVER_DRIVABLE:
    case Label::UNDER_DRIVABLE:
      return PerceptionLabel::ROAD_OBSTACLE;
    case Label::UNKNOWN:
    default:
      return PerceptionLabel::UNKNOWN;
  }
}

perception_msgs::msg::ObjectClassification toPerceptionClassification(
  const autoware_perception_msgs::msg::ObjectClassification & classification)
{
  perception_msgs::msg::ObjectClassification output_classification;
  output_classification.type = toPerceptionClassificationType(classification.label);
  output_classification.probability = classification.probability;
  return output_classification;
}

}  // namespace

void box3DToDetectedObject(
  const Box3D & box3d, const std::vector<std::string> & class_names, const bool has_twist,
  const bool has_variance, autoware_perception_msgs::msg::DetectedObject & obj)
{
  // TODO(yukke42): the value of classification confidence of DNN, not probability.
  obj.existence_probability = box3d.score;

  // classification
  autoware_perception_msgs::msg::ObjectClassification classification;
  classification.probability = 1.0f;
  if (box3d.label >= 0 && static_cast<size_t>(box3d.label) < class_names.size()) {
    classification.label = getSemanticType(class_names[box3d.label]);
  } else {
    classification.label = Label::UNKNOWN;
    RCLCPP_WARN_STREAM(
      rclcpp::get_logger("lidar_centerpoint"), "Unexpected label: UNKNOWN is set.");
  }

  if (autoware::object_recognition_utils::isCarLikeVehicle(classification.label)) {
    obj.kinematics.orientation_availability =
      autoware_perception_msgs::msg::DetectedObjectKinematics::SIGN_UNKNOWN;
  }

  obj.classification.emplace_back(classification);

  // pose and shape
  // mmdet3d yaw format to ros yaw format
  const float yaw = -box3d.yaw - autoware_utils::pi / 2;
  obj.kinematics.pose_with_covariance.pose.position =
    autoware_utils::create_point(box3d.x, box3d.y, box3d.z);
  obj.kinematics.pose_with_covariance.pose.orientation =
    autoware_utils::create_quaternion_from_yaw(yaw);
  obj.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  obj.shape.dimensions =
    autoware_utils::create_translation(box3d.length, box3d.width, box3d.height);
  if (has_variance) {
    obj.kinematics.has_position_covariance = has_variance;
    obj.kinematics.pose_with_covariance.covariance = convertPoseCovarianceMatrix(box3d);
  }

  // twist
  if (has_twist) {
    float vel_x = box3d.vel_x;
    float vel_y = box3d.vel_y;

    // twist of the object is based on the object coordinate system
    geometry_msgs::msg::Twist twist;
    twist.linear.x = std::cos(yaw) * vel_x + std::sin(yaw) * vel_y;
    twist.linear.y = -std::sin(yaw) * vel_x + std::cos(yaw) * vel_y;
    twist.angular.z = 0;  // angular velocity is not supported

    obj.kinematics.twist_with_covariance.twist = twist;
    obj.kinematics.has_twist = has_twist;
    if (has_variance) {
      obj.kinematics.has_twist_covariance = has_variance;
      obj.kinematics.twist_with_covariance.covariance = convertTwistCovarianceMatrix(box3d, yaw);
    }
  }
}

perception_msgs::msg::ObjectList detectedObjectsToObjectList(
  const autoware_perception_msgs::msg::DetectedObjects & objects_msg)
{
  perception_msgs::msg::ObjectList output_msg;
  output_msg.header = objects_msg.header;
  output_msg.objects.reserve(objects_msg.objects.size());

  for (std::size_t index = 0; index < objects_msg.objects.size(); ++index) {
    const auto & input_object = objects_msg.objects.at(index);

    perception_msgs::msg::Object output_object;
    output_object.id = static_cast<uint64_t>(index + 1);
    output_object.existence_probability = input_object.existence_probability;

    perception_msgs::object_access::initializeState(
      output_object, perception_msgs::msg::ISCACTR::MODEL_ID);
    output_object.state.header = objects_msg.header;
    output_object.state.reference_point.value =
      perception_msgs::msg::ObjectReferencePoint::GEOMETRIC_CENTER;

    if (input_object.classification.empty()) {
      perception_msgs::msg::ObjectClassification classification;
      classification.type = perception_msgs::msg::ObjectClassification::UNKNOWN;
      classification.probability = 0.0;
      output_object.state.classifications.push_back(classification);
    } else {
      output_object.state.classifications.reserve(input_object.classification.size());
      for (const auto & classification : input_object.classification) {
        output_object.state.classifications.push_back(toPerceptionClassification(classification));
      }
    }

    if (input_object.kinematics.has_position_covariance) {
      perception_msgs::object_access::setPoseWithCovariance(
        output_object, input_object.kinematics.pose_with_covariance);
    } else {
      perception_msgs::object_access::setPose(
        output_object, input_object.kinematics.pose_with_covariance.pose);
    }

    perception_msgs::object_access::setLength(output_object, input_object.shape.dimensions.x);
    perception_msgs::object_access::setWidth(output_object, input_object.shape.dimensions.y);
    perception_msgs::object_access::setHeight(output_object, input_object.shape.dimensions.z);

    if (input_object.kinematics.has_twist) {
      geometry_msgs::msg::Vector3 velocity;
      velocity.x = input_object.kinematics.twist_with_covariance.twist.linear.x;
      velocity.y = input_object.kinematics.twist_with_covariance.twist.linear.y;
      velocity.z = input_object.kinematics.twist_with_covariance.twist.linear.z;
      perception_msgs::object_access::setVelocity(output_object, velocity, false);

      if (input_object.kinematics.has_twist_covariance) {
        constexpr auto model_size = perception_msgs::msg::ISCACTR::CONTINUOUS_STATE_SIZE;
        constexpr auto vel_lon = perception_msgs::msg::ISCACTR::VEL_LON;
        constexpr auto vel_lat = perception_msgs::msg::ISCACTR::VEL_LAT;
        const auto & covariance = input_object.kinematics.twist_with_covariance.covariance;

        output_object.state.continuous_state_covariance[model_size * vel_lon + vel_lon] =
          covariance.at(0);
        output_object.state.continuous_state_covariance[model_size * vel_lon + vel_lat] =
          covariance.at(1);
        output_object.state.continuous_state_covariance[model_size * vel_lat + vel_lon] =
          covariance.at(6);
        output_object.state.continuous_state_covariance[model_size * vel_lat + vel_lat] =
          covariance.at(7);
      }
    }

    output_msg.objects.push_back(std::move(output_object));
  }

  return output_msg;
}

uint8_t getSemanticType(const std::string & class_name)
{
  if (class_name == "CAR") {
    return Label::CAR;
  } else if (class_name == "TRUCK") {
    return Label::TRUCK;
  } else if (class_name == "BUS") {
    return Label::BUS;
  } else if (class_name == "TRAILER") {
    return Label::TRAILER;
  } else if (class_name == "BICYCLE") {
    return Label::BICYCLE;
  } else if (class_name == "MOTORBIKE") {
    return Label::MOTORCYCLE;
  } else if (class_name == "PEDESTRIAN") {
    return Label::PEDESTRIAN;
  } else {
    return Label::UNKNOWN;
  }
}

std::array<double, 36> convertPoseCovarianceMatrix(const Box3D & box3d)
{
  using POSE_IDX = autoware_utils::xyzrpy_covariance_index::XYZRPY_COV_IDX;
  std::array<double, 36> pose_covariance{};
  pose_covariance[POSE_IDX::X_X] = box3d.x_variance;
  pose_covariance[POSE_IDX::Y_Y] = box3d.y_variance;
  pose_covariance[POSE_IDX::Z_Z] = box3d.z_variance;
  pose_covariance[POSE_IDX::YAW_YAW] = box3d.yaw_variance;
  return pose_covariance;
}

std::array<double, 36> convertTwistCovarianceMatrix(const Box3D & box3d, const float yaw)
{
  using POSE_IDX = autoware_utils::xyzrpy_covariance_index::XYZRPY_COV_IDX;

  // twist covariance matrix is based on the object coordinate system
  std::array<double, 36> twist_covariance{};
  const float cos_yaw = std::cos(yaw);
  const float sin_yaw = std::sin(yaw);
  twist_covariance[POSE_IDX::X_X] =
    box3d.vel_x_variance * cos_yaw * cos_yaw + box3d.vel_y_variance * sin_yaw * sin_yaw;
  twist_covariance[POSE_IDX::X_Y] =
    (box3d.vel_y_variance - box3d.vel_x_variance) * sin_yaw * cos_yaw;
  twist_covariance[POSE_IDX::Y_X] = twist_covariance[POSE_IDX::X_Y];
  twist_covariance[POSE_IDX::Y_Y] =
    box3d.vel_x_variance * sin_yaw * sin_yaw + box3d.vel_y_variance * cos_yaw * cos_yaw;
  return twist_covariance;
}

}  // namespace autoware::lidar_centerpoint
