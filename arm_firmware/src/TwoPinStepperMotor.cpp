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

long TwoPinStepperMotor::getPosition() const
{
    return accelStepper->currentPosition();
}

void TwoPinStepperMotor::moveTo(long position)
{
    accelStepper->moveTo(position);
}

void TwoPinStepperMotor::run()
{
    accelStepper->run();
}
