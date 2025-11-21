// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from aruco_opencv_msgs:msg/ArucoDetection.idl
// generated code does not contain a copyright notice
#ifndef ARUCO_OPENCV_MSGS__MSG__DETAIL__ARUCO_DETECTION__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define ARUCO_OPENCV_MSGS__MSG__DETAIL__ARUCO_DETECTION__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "aruco_opencv_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "aruco_opencv_msgs/msg/detail/aruco_detection__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_aruco_opencv_msgs
bool cdr_serialize_aruco_opencv_msgs__msg__ArucoDetection(
  const aruco_opencv_msgs__msg__ArucoDetection * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_aruco_opencv_msgs
bool cdr_deserialize_aruco_opencv_msgs__msg__ArucoDetection(
  eprosima::fastcdr::Cdr &,
  aruco_opencv_msgs__msg__ArucoDetection * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_aruco_opencv_msgs
size_t get_serialized_size_aruco_opencv_msgs__msg__ArucoDetection(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_aruco_opencv_msgs
size_t max_serialized_size_aruco_opencv_msgs__msg__ArucoDetection(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_aruco_opencv_msgs
bool cdr_serialize_key_aruco_opencv_msgs__msg__ArucoDetection(
  const aruco_opencv_msgs__msg__ArucoDetection * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_aruco_opencv_msgs
size_t get_serialized_size_key_aruco_opencv_msgs__msg__ArucoDetection(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_aruco_opencv_msgs
size_t max_serialized_size_key_aruco_opencv_msgs__msg__ArucoDetection(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_aruco_opencv_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, aruco_opencv_msgs, msg, ArucoDetection)();

#ifdef __cplusplus
}
#endif

#endif  // ARUCO_OPENCV_MSGS__MSG__DETAIL__ARUCO_DETECTION__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
