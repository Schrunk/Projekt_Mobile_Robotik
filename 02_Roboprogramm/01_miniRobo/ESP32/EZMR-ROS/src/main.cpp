#include <micro_ros_arduino.h>

rclc_executor_t executor;
rcl_subscription_t subscriber;
rclc_support_t support;
rcl_node_t node;

void setup() {
  Serial.begin(115200);

  // Custom transport (UART)
  rmw_uros_set_custom_transport(
    true,
    NULL,
    uart_transport_open,
    uart_transport_close,
    uart_transport_write,
    uart_transport_read
  );

  // Init allocator und support
  rcl_allocator_t allocator = rcutils_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);

  // Node und Subscription
  rclc_node_init_default(&node, "esp32_node", "", &support);
  rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "topic"
  );

  // Executor
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, NULL, callback, ON_NEW_DATA);
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}
