#include "robot-config.h"
          
void driveTurnRightForUsingGyro(float targetRotation=90, float targetVelocity=40)
{
    float gyroValue;
    float oldValue;
    float runningTotal;
    float newValue;
    float error = 2.3;
    float minVelocityRatio = 0.5;
    float dampingFactor = 0;
    float velocity = 0;
    
    gyroValue = Gyro1.value(rotationUnits::deg);
    oldValue=gyroValue;
    runningTotal=0;

    while(runningTotal<=(targetRotation-error))
    {                                
        dampingFactor = 1 - (minVelocityRatio * runningTotal/targetRotation);
        velocity = targetVelocity * dampingFactor;
        Leftfront.spin(directionType::fwd,velocity,velocityUnits::pct);
        Leftback.spin(directionType::fwd,velocity,velocityUnits::pct);
        Rightfront.spin(directionType::rev,velocity,velocityUnits::pct);
        Rightback.spin(directionType::rev,velocity,velocityUnits::pct);
        newValue = Gyro1.value(rotationUnits::deg);
        if (newValue<oldValue)
        {
            // Passed 360deg, so add back in 360
            newValue += 360;
        }
        runningTotal += newValue - oldValue;
        oldValue = newValue;                
        task::sleep(5);           
    }

    Leftfront.stop(brakeType::brake);
    Leftback.stop(brakeType::brake);
    Rightfront.stop(brakeType::brake);
    Rightback.stop(brakeType::brake);
}


void driveTurnLeftForUsingGyro(float targetRotation=90, float targetVelocity=40)
{
    float gyroValue;
    float oldValue;
    float runningTotal;
    float newValue;
    float error = 2.3;
    float minVelocityRatio = 0.5;
    float velocity = 0;
    float dampingFactor=0;
    
    gyroValue = Gyro1.value(rotationUnits::deg);
    oldValue=gyroValue;
    runningTotal=0;
                       
    while(runningTotal<=(targetRotation-error))
    {                
        dampingFactor = 1 - (minVelocityRatio * runningTotal/targetRotation);
        velocity = targetVelocity * dampingFactor;
        Leftfront.spin(directionType::rev,velocity,velocityUnits::pct);
        Leftback.spin(directionType::rev,velocity,velocityUnits::pct);
        Rightfront.spin(directionType::fwd,velocity,velocityUnits::pct);
        Rightback.spin(directionType::fwd,velocity,velocityUnits::pct);
        if (newValue>oldValue)
        {
            // Passed 360deg, so remove 360
            newValue -= 360;
        }
        newValue = Gyro1.value(rotationUnits::deg);
        runningTotal += oldValue - newValue;
        oldValue = newValue;
        task::sleep(5);                              
    }

    Leftfront.stop(brakeType::brake);
    Leftback.stop(brakeType::brake);
    Rightfront.stop(brakeType::brake);
    Rightback.stop(brakeType::brake);

}
