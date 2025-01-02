
#include <micro_ros_platformio.h>
#include <Arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <arm_interfaces/msg/motors.h>
#include "TwoPinStepperMotor.h"

rcl_publisher_t publisher;
rcl_subscription_t subscriber;
arm_interfaces__msg__Motors msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

const int baseLink_step = 12;
const int baseLink_dir = 14;

// TwoPinStepperMotor baseLink(baseLink_step,baseLink_dir);

#define RCSOFTCHECK(fn)                \
    {                                  \
        rcl_ret_t temp_rc = fn;        \
        if ((temp_rc != RCL_RET_OK))   \
        {                              \
            printf("NOOOOOOOOOOOOOO"); \
        }                              \
    }

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        arm_interfaces__msg__Motors motorsState;

        // motorsState.base_link = baseLink.getPosition();
        motorsState.base_link = 12;
        RCSOFTCHECK(rcl_publish(&publisher, &motorsState, NULL));
    }
}

void subscription_callback(const void *msgin)
{
    // Cast received message to used type
    const arm_interfaces__msg__Motors *command = (const arm_interfaces__msg__Motors *)msgin;

    //   // Process message
    printf("base_link: %d\n", command->base_link);
}

void setup()
{

    Serial.begin(115200);
    set_microros_serial_transports(Serial);

    allocator = rcl_get_default_allocator();

    // create init_options
    RCSOFTCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // create node
    RCSOFTCHECK(rclc_node_init_default(&node, "micro_ros_arm_motors", "", &support));

    const rosidl_message_type_support_t *type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(arm_interfaces, msg,
                                                                                    Motors);

    // create publisher
    RCSOFTCHECK(rclc_publisher_init_default(
        &publisher,
        &node,
        type_support,
        "arm/motors_state"));

    RCSOFTCHECK(rclc_subscription_init_default(
        &subscriber,
        &node,
        type_support,
        "arm/motors_cmd"));

    RCSOFTCHECK(rclc_executor_add_subscription(
        &executor,
        &subscriber,
        &msg,
        &subscription_callback,
        ON_NEW_DATA));

    // create timer,
    const unsigned int timer_timeout = 1000;
    RCSOFTCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        timer_callback));

    // create executor
    RCSOFTCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCSOFTCHECK(rclc_executor_add_timer(&executor, &timer));
}

void loop()
{
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}