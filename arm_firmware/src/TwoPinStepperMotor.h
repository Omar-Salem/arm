//
// Created by omar on 4/2/24.
//

#ifndef ARM_STEPPERMOTOR_H
#define ARM_STEPPERMOTOR_H

#include <AccelStepper.h>

class TwoPinStepperMotor {
private:
    AccelStepper *accelStepper;

public:
    TwoPinStepperMotor(int stepPin, int dirPin);

    long getPosition() const;

    void moveTo(long position);

    void run();
};


#endif //ARM_STEPPERMOTOR_H
