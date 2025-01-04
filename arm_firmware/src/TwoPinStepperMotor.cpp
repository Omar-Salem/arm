//
// Created by omar on 4/2/24.
//

#include "TwoPinStepperMotor.h"

TwoPinStepperMotor::TwoPinStepperMotor(int stepPin,
                                       int dirPin)
{
    accelStepper = new AccelStepper(AccelStepper::DRIVER, stepPin, dirPin);
    accelStepper->setMaxSpeed(200);
}

double TwoPinStepperMotor::getPosition() const
{
    return accelStepper->currentPosition() * STEPS_TO_RADIANS;
}

void TwoPinStepperMotor::moveTo(double position)
{
    accelStepper->moveTo(position * RADIANS_TO_STEPS);
}

void TwoPinStepperMotor::run()
{
    accelStepper->run();
}
