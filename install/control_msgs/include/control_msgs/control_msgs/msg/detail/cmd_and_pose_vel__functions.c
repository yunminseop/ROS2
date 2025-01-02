// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from control_msgs:msg/CmdAndPoseVel.idl
// generated code does not contain a copyright notice
#include "control_msgs/msg/detail/cmd_and_pose_vel__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
control_msgs__msg__CmdAndPoseVel__init(control_msgs__msg__CmdAndPoseVel * msg)
{
  if (!msg) {
    return false;
  }
  // cmd_vel_linear
  // cmd_vel_angular
  // pose_x
  // pose_y
  // linear_vel
  // angular_vel
  return true;
}

void
control_msgs__msg__CmdAndPoseVel__fini(control_msgs__msg__CmdAndPoseVel * msg)
{
  if (!msg) {
    return;
  }
  // cmd_vel_linear
  // cmd_vel_angular
  // pose_x
  // pose_y
  // linear_vel
  // angular_vel
}

bool
control_msgs__msg__CmdAndPoseVel__are_equal(const control_msgs__msg__CmdAndPoseVel * lhs, const control_msgs__msg__CmdAndPoseVel * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // cmd_vel_linear
  if (lhs->cmd_vel_linear != rhs->cmd_vel_linear) {
    return false;
  }
  // cmd_vel_angular
  if (lhs->cmd_vel_angular != rhs->cmd_vel_angular) {
    return false;
  }
  // pose_x
  if (lhs->pose_x != rhs->pose_x) {
    return false;
  }
  // pose_y
  if (lhs->pose_y != rhs->pose_y) {
    return false;
  }
  // linear_vel
  if (lhs->linear_vel != rhs->linear_vel) {
    return false;
  }
  // angular_vel
  if (lhs->angular_vel != rhs->angular_vel) {
    return false;
  }
  return true;
}

bool
control_msgs__msg__CmdAndPoseVel__copy(
  const control_msgs__msg__CmdAndPoseVel * input,
  control_msgs__msg__CmdAndPoseVel * output)
{
  if (!input || !output) {
    return false;
  }
  // cmd_vel_linear
  output->cmd_vel_linear = input->cmd_vel_linear;
  // cmd_vel_angular
  output->cmd_vel_angular = input->cmd_vel_angular;
  // pose_x
  output->pose_x = input->pose_x;
  // pose_y
  output->pose_y = input->pose_y;
  // linear_vel
  output->linear_vel = input->linear_vel;
  // angular_vel
  output->angular_vel = input->angular_vel;
  return true;
}

control_msgs__msg__CmdAndPoseVel *
control_msgs__msg__CmdAndPoseVel__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  control_msgs__msg__CmdAndPoseVel * msg = (control_msgs__msg__CmdAndPoseVel *)allocator.allocate(sizeof(control_msgs__msg__CmdAndPoseVel), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(control_msgs__msg__CmdAndPoseVel));
  bool success = control_msgs__msg__CmdAndPoseVel__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
control_msgs__msg__CmdAndPoseVel__destroy(control_msgs__msg__CmdAndPoseVel * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    control_msgs__msg__CmdAndPoseVel__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
control_msgs__msg__CmdAndPoseVel__Sequence__init(control_msgs__msg__CmdAndPoseVel__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  control_msgs__msg__CmdAndPoseVel * data = NULL;

  if (size) {
    data = (control_msgs__msg__CmdAndPoseVel *)allocator.zero_allocate(size, sizeof(control_msgs__msg__CmdAndPoseVel), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = control_msgs__msg__CmdAndPoseVel__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        control_msgs__msg__CmdAndPoseVel__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
control_msgs__msg__CmdAndPoseVel__Sequence__fini(control_msgs__msg__CmdAndPoseVel__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      control_msgs__msg__CmdAndPoseVel__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

control_msgs__msg__CmdAndPoseVel__Sequence *
control_msgs__msg__CmdAndPoseVel__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  control_msgs__msg__CmdAndPoseVel__Sequence * array = (control_msgs__msg__CmdAndPoseVel__Sequence *)allocator.allocate(sizeof(control_msgs__msg__CmdAndPoseVel__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = control_msgs__msg__CmdAndPoseVel__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
control_msgs__msg__CmdAndPoseVel__Sequence__destroy(control_msgs__msg__CmdAndPoseVel__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    control_msgs__msg__CmdAndPoseVel__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
control_msgs__msg__CmdAndPoseVel__Sequence__are_equal(const control_msgs__msg__CmdAndPoseVel__Sequence * lhs, const control_msgs__msg__CmdAndPoseVel__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!control_msgs__msg__CmdAndPoseVel__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
control_msgs__msg__CmdAndPoseVel__Sequence__copy(
  const control_msgs__msg__CmdAndPoseVel__Sequence * input,
  control_msgs__msg__CmdAndPoseVel__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(control_msgs__msg__CmdAndPoseVel);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    control_msgs__msg__CmdAndPoseVel * data =
      (control_msgs__msg__CmdAndPoseVel *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!control_msgs__msg__CmdAndPoseVel__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          control_msgs__msg__CmdAndPoseVel__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!control_msgs__msg__CmdAndPoseVel__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
