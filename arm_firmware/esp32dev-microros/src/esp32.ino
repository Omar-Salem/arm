// https://www.hackster.io/514301/micro-ros-on-esp32-using-arduino-ide-1360ca
// https://github.com/micro-ROS/micro_ros_platformio

#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <arm_interfaces/msg/motors.h>
#include "TwoPinStepperMotor.h"

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

rcl_subscription_t positionCommandSubscriber;
arm_interfaces__msg__Motors positionCommandCallbackMessage;
rclc_executor_t subscriberExecutor;

rcl_publisher_t statePublisher;
rclc_executor_t publisherExecutor;

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t publisherTimer;
const unsigned int PUBLISHER_TIMER_TIMEOUT_MILL = 100;

const int baseLink_step = 18;
const int baseLink_dir = 19;

const int shoulder_step = 4;
const int shoulder_dir = 16;

TwoPinStepperMotor baseLink(baseLink_step,baseLink_dir);
TwoPinStepperMotor shoulder(shoulder_step, shoulder_dir);

// https://randomnerdtutorials.com/esp32-dual-core-arduino-ide/
TaskHandle_t moveMotorsTask;


// #define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handle loop
// void error_loop() {
//     while (1) {
//         delay(100);
//     }
// }

void positionCommandCallback(const void *msgin) {
    const arm_interfaces__msg__Motors *command = (const arm_interfaces__msg__Motors *) msgin;
    baseLink.setPosition(command->base_link);
    shoulder.setPosition(command->shoulder);
}

void stateTimerCallback(rcl_timer_t *timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        arm_interfaces__msg__Motors msg;

        msg.base_link = baseLink.getPosition();
        msg.shoulder = shoulder.getPosition();
        
        RCSOFTCHECK(rcl_publish(&statePublisher, &msg, NULL));
    }
}

void createStatePublisher() {

    const rosidl_message_type_support_t *type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(arm_interfaces, msg,
                                                                                    Motors);


    RCSOFTCHECK(rclc_publisher_init_default(
            &statePublisher,
            &node,
            type_support,
            "arm/motors_state"));

    // create timer
    RCSOFTCHECK(rclc_timer_init_default(
            &publisherTimer,
            &support,
            RCL_MS_TO_NS(PUBLISHER_TIMER_TIMEOUT_MILL),
            stateTimerCallback));


    // create executor
    RCSOFTCHECK(rclc_executor_init(&publisherExecutor, &support.context, 1, &allocator));
    RCSOFTCHECK(rclc_executor_add_timer(&publisherExecutor, &publisherTimer));
}

void createCommandSubscriber() {
    const rosidl_message_type_support_t *type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(arm_interfaces, msg,
                                                                                    Motors);

    RCSOFTCHECK(rclc_subscription_init_default(
            &positionCommandSubscriber,
            &node,
            type_support,
            "arm/motors_cmd"));


    RCSOFTCHECK(rclc_executor_init(&subscriberExecutor, &support.context, 1, &allocator));

    RCSOFTCHECK(rclc_executor_add_subscription(&subscriberExecutor,
                                               &positionCommandSubscriber,
                                               &positionCommandCallbackMessage,
                                               &positionCommandCallback,
                                               ON_NEW_DATA));
}

void setup() {
    Serial.begin(115200);
    set_microros_serial_transports(Serial);

    allocator = rcl_get_default_allocator();

    RCSOFTCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    RCSOFTCHECK(rclc_node_init_default(&node, "micro_ros_arm_motors", "", &support));

    createStatePublisher();

    createCommandSubscriber();

    xTaskCreatePinnedToCore(
            moveMotors,   /* Task function. */
            "moveMotorsTask",     /* name of task. */
            10000,       /* Stack size of task */
            NULL,        /* parameter of the task */
            0,           /* priority of the task */
            &moveMotorsTask,      /* Task handle to keep track of created task */
            0);          /* pin task to core 1 */

}

void loop() {
    RCSOFTCHECK(rclc_executor_spin_some(&publisherExecutor, RCL_MS_TO_NS(50)));
    RCSOFTCHECK(rclc_executor_spin_some(&subscriberExecutor, RCL_MS_TO_NS(50)));
}

void moveMotors(void *pvParameters) {
    for (;;) {
        baseLink.move();
        shoulder.move();
    }
}