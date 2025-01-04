//
// Created by omar on 4/2/24.
//

#include "TwoPinStepperMotor.h"

TwoPinStepperMotor::TwoPinStepperMotor(int stepPin,
                                       int dirPin,
                                       double reduction)
{
    accelStepper = new AccelStepper(AccelStepper::DRIVER, stepPin, dirPin);
    accelStepper->setMaxSpeed(MAX_SPEED);
    this->reduction = reduction;
}

double TwoPinStepperMotor::getPosition() const
{
    return accelStepper->currentPosition() * STEPS_TO_RADIANS / reduction;
}

void TwoPinStepperMotor::moveTo(double position)
{
    accelStepper->moveTo(position * RADIANS_TO_STEPS * reduction);
    accelStepper->setSpeed(FULL_CIRCLE_STEPS * 2);
}

void TwoPinStepperMotor::run()
{
    accelStepper->runSpeedToPosition();
}
