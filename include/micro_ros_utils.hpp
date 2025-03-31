#pragma once
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/int32.h>
#include <trajectory_msgs/msg/joint_trajectory_point.h>

#include <params.hpp>

#define RCCHECK(fn)                    \
    {                                  \
        rcl_ret_t temp_rc = fn;        \
        if ((temp_rc != RCL_RET_OK)) { \
            return false;              \
        }                              \
    }
#define RCSOFTCHECK(fn)                \
    {                                  \
        rcl_ret_t temp_rc = fn;        \
        if ((temp_rc != RCL_RET_OK)) { \
        }                              \
    }
#define FNCHECK(fn, type_) \
    {                      \
        type_ ret = fn;    \
        if (!ret) {        \
            return false;  \
        }                  \
    }
#define EXECUTE_EVERY_N_MS(MS, X)          \
    do {                                   \
        static volatile int64_t init = -1; \
        if (init == -1) {                  \
            init = uxr_millis();           \
        }                                  \
        if (uxr_millis() - init > MS) {    \
            X;                             \
            init = uxr_millis();           \
        }                                  \
    } while (0)

bool create_node(rcl_allocator_t *allocator,
                 rcl_init_options_t *init_options,
                 rclc_support_t *support,
                 rcl_node_t *node,
                 size_t domain_id = ROS_DOMAIN_ID,
                 const char *node_name = NODE_NAME,
                 const char *namespace_ = NAMESPACE);

bool create_subscriber(rcl_allocator_t *allocator,
                       rclc_support_t *support,
                       rcl_node_t *node,
                       const char *topic_name,
                       rcl_subscription_t *subscription,
                       rclc_subscription_callback_t callback,
                       trajectory_msgs__msg__JointTrajectoryPoint *data,
                       const size_t data_size,
                       rclc_executor_t *executor,
                       const rclc_executor_handle_invocation_t invocation = ON_NEW_DATA);

bool create_publisher(rcl_allocator_t *allocator,
                      rclc_support_t *support,
                      rcl_node_t *node,
                      const char *topic_name,
                      rcl_publisher_t *publisher,
                      rcl_timer_t *timer,
                      rcl_timer_callback_t callback,
                      trajectory_msgs__msg__JointTrajectoryPoint *data,
                      const size_t data_size,
                      rclc_executor_t *executor,
                      const unsigned int timer_timeout = 50);

void delete_node(rcl_init_options_t *init_options,
                 rclc_support_t *support,
                 rcl_node_t *node);

void delete_subscriber(rcl_node_t *node,
                       rcl_subscription_t *subscription,
                       trajectory_msgs__msg__JointTrajectoryPoint *data,
                       rclc_executor_t *executor);

void delete_publisher(rcl_node_t *node,
                      rcl_publisher_t *publisher,
                      rcl_timer_t *timer,
                      trajectory_msgs__msg__JointTrajectoryPoint *data,
                      rclc_executor_t *executor);
