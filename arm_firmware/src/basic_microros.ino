
// #include <micro_ros_platformio.h>
// #include <Arduino.h>
// #include <stdio.h>
// #include <rcl/rcl.h>
// #include <rcl/error_handling.h>
// #include <rclc/rclc.h>
// #include <rclc/executor.h>

// #include <std_msgs/msg/int32.h>

// rcl_publisher_t publisher;
// std_msgs__msg__Int32 msg;
// rclc_executor_t executor;
// rclc_support_t support;
// rcl_allocator_t allocator;
// rcl_node_t node;
// rcl_timer_t timer;

// #define RCSOFTCHECK(fn)                \
//     {                                  \
//         rcl_ret_t temp_rc = fn;        \
//         if ((temp_rc != RCL_RET_OK))   \
//         {                              \
//             printf("NOOOOOOOOOOOOOO"); \
//         }                              \
//     }

// void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
// {
//     RCLC_UNUSED(last_call_time);
//     if (timer != NULL)
//     {
//         RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
//         msg.data++;
//     }
// }

// void setup()
// {

//     Serial.begin(115200);
//     set_microros_serial_transports(Serial);

//     allocator = rcl_get_default_allocator();

//     // create init_options
//     RCSOFTCHECK(rclc_support_init(&support, 0, NULL, &allocator));

//     // create node
//     RCSOFTCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

//     // create publisher
//     RCSOFTCHECK(rclc_publisher_init_default(
//         &publisher,
//         &node,
//         ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
//         "micro_ros_arduino_node_publisher"));

//     // create timer,
//     const unsigned int timer_timeout = 1000;
//     RCSOFTCHECK(rclc_timer_init_default(
//         &timer,
//         &support,
//         RCL_MS_TO_NS(timer_timeout),
//         timer_callback));

//     // create executor
//     RCSOFTCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
//     RCSOFTCHECK(rclc_executor_add_timer(&executor, &timer));

//     msg.data = 0;
// }

// void loop()
// {
//     RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
// }