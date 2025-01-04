//
// Created by omar on 4/2/24.
//

#ifndef ARM_STEPPERMOTOR_H
#define ARM_STEPPERMOTOR_H

#include <AccelStepper.h>

class TwoPinStepperMotor
{
private:
    const double FULL_CIRCLE_RADIANS = 6.28319;
    const double FULL_CIRCLE_STEPS = 200;
    const double RADIANS_TO_STEPS = FULL_CIRCLE_STEPS / FULL_CIRCLE_RADIANS;
    const double STEPS_TO_RADIANS = FULL_CIRCLE_RADIANS / FULL_CIRCLE_STEPS;
    const int MAX_SPEED = 1000;
    double reduction;
    AccelStepper *accelStepper;

public:
    TwoPinStepperMotor(int stepPin, int dirPin, double reduction = 1);

    double getPosition() const;

    void moveTo(double position);

    void run();
};

#endif // ARM_STEPPERMOTOR_H
