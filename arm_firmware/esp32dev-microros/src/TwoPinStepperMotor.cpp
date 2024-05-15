//
// Created by omar on 4/2/24.
//

#include "TwoPinStepperMotor.h"

TwoPinStepperMotor::TwoPinStepperMotor(byte stepPin,
                                       byte dirPin) {
    commandLastPing_ = millis();
    accelStepper = new AccelStepper(1, stepPin);
}


double TwoPinStepperMotor::getPosition() const {
    return accelStepper->currentPosition();
}

void TwoPinStepperMotor::setPosition(double position) {
    commandLastPing_ = millis();
    accelStepper->setPosition(position);
}

void TwoPinStepperMotor::move() {
    // if (millis() - commandLastPing_ >= DEAD_MAN_SWITCH_TIMEOUT_MILLI_SEC) {
    //     accelStepper->setSpeed(0);
    // }
    accelStepper->runSpeed();
}
