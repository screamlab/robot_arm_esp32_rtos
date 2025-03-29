#include <micro_ros_utils.hpp>

bool create_node(rcl_allocator_t *allocator,
                 rcl_init_options_t *init_options,
                 rclc_support_t *support,
                 rcl_node_t *node,
                 size_t domain_id,
                 const char *node_name,
                 const char *namespace_) {
    // Initialize micro-ROS allocator
    *allocator = rcl_get_default_allocator();

    // Initialize and modify options (Set DOMAIN ID)
    *init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(init_options, *allocator));
    RCCHECK(rcl_init_options_set_domain_id(init_options, domain_id));

    // create init_options
    RCCHECK(rclc_support_init_with_options(support, 0, NULL, init_options, allocator));

    // create node
    RCCHECK(rclc_node_init_default(node, node_name, namespace_, support));

    return true;
}

bool create_subscriber(rcl_allocator_t *allocator,
                       rclc_support_t *support,
                       rcl_node_t *node,
                       const char *topic_name,
                       rcl_subscription_t *subscription,
                       rclc_subscription_callback_t callback,
                       trajectory_msgs__msg__JointTrajectoryPoint *data,
                       const size_t data_size,
                       rclc_executor_t *executor,
                       const rclc_executor_handle_invocation_t invocation) {
    // create subscriber
    RCCHECK(rclc_subscription_init_default(
        subscription,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(trajectory_msgs, msg, JointTrajectoryPoint),
        topic_name));

    data->positions.capacity = data_size;
    data->positions.size = data_size;
    FNCHECK(data->positions.data = (double *)calloc(data->positions.capacity, sizeof(double)), double *);

    // create subscriber executor
    RCCHECK(rclc_executor_init(executor, &support->context, 1, allocator));
    RCCHECK(rclc_executor_add_subscription(executor, subscription, data, callback, invocation));

    return true;
}

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
                      const unsigned int timer_timeout) {
    // Create publisher
    RCCHECK(rclc_publisher_init_default(
        publisher,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(trajectory_msgs, msg, JointTrajectoryPoint),
        topic_name));

    // Create timer
    RCCHECK(rclc_timer_init_default(
        timer,
        support,
        RCL_MS_TO_NS(timer_timeout),  // Timer period in nanoseconds
        callback));

    data->positions.capacity = data_size;
    data->positions.size = data_size;
    FNCHECK(data->positions.data = (double *)calloc(data->positions.capacity, sizeof(double)), double *);

    // Create publisher executor
    RCCHECK(rclc_executor_init(executor, &support->context, 1, allocator));
    RCCHECK(rclc_executor_add_timer(executor, timer));

    return true;
}

void delete_node(rcl_init_options_t *init_options,
                 rclc_support_t *support,
                 rcl_node_t *node) {
    // Free resources
    RCSOFTCHECK(rcl_node_fini(node));
    RCSOFTCHECK(rclc_support_fini(support));
    RCSOFTCHECK(rcl_init_options_fini(init_options));
}

void delete_subscriber(rcl_node_t *node,
                       rcl_subscription_t *subscription,
                       trajectory_msgs__msg__JointTrajectoryPoint *data,
                       rclc_executor_t *executor) {
    // Free resources
    RCSOFTCHECK(rcl_subscription_fini(subscription, node));
    RCSOFTCHECK(rclc_executor_fini(executor));
    free(data->positions.data);
    data->positions.data = NULL;
}

void delete_publisher(rcl_node_t *node,
                      rcl_publisher_t *publisher,
                      rcl_timer_t *timer,
                      trajectory_msgs__msg__JointTrajectoryPoint *data,
                      rclc_executor_t *executor) {
    // Free resources
    RCSOFTCHECK(rcl_publisher_fini(publisher, node));
    RCSOFTCHECK(rcl_timer_fini(timer));
    RCSOFTCHECK(rclc_executor_fini(executor));
    free(data->positions.data);
    data->positions.data = NULL;
}