
#ifndef FlexyStepper_h
#define FlexyStepper_h

#include <stdlib.h>
#include <wiringPi.h>
#include <math.h> 

//
// the FlexyStepper class
//
class FlexyStepper
{
  public:
    //
    // public functions
    //
    FlexyStepper();
    void connectToPins(int stepPinNumber, int directionPinNumber);

    void setStepsPerMillimeter(float motorStepPerMillimeter);
    float getCurrentPositionInMillimeters();
    void setCurrentPositionInMillimeters(float currentPositionInMillimeters);
    void setCurrentPositionInMillimeter(float currentPositionInMillimeter);
    void setSpeedInMillimetersPerSecond(float speedInMillimetersPerSecond);
    void setAccelerationInMillimetersPerSecondPerSecond(float accelerationInMillimetersPerSecondPerSecond);
    bool moveToHomeInMillimeters(long directionTowardHome, float speedInMillimetersPerSecond, long maxDistanceToMoveInMillimeters, int homeLimitSwitchPin);
    void moveRelativeInMillimeters(float distanceToMoveInMillimeters);
    void setTargetPositionRelativeInMillimeters(float distanceToMoveInMillimeters);
    void moveToPositionInMillimeters(float absolutePositionToMoveToInMillimeters);
    void setTargetPositionInMillimeters(float absolutePositionToMoveToInMillimeters);
    float getCurrentVelocityInMillimetersPerSecond();
   
    void setStepsPerRevolution(float motorStepPerRevolution);
    void setCurrentPositionInRevolutions(float currentPositionInRevolutions);
    float getCurrentPositionInRevolutions();
    void setSpeedInRevolutionsPerSecond(float speedInRevolutionsPerSecond);
    void setAccelerationInRevolutionsPerSecondPerSecond(float accelerationInRevolutionsPerSecondPerSecond);
    bool moveToHomeInRevolutions(long directionTowardHome, float speedInRevolutionsPerSecond, long maxDistanceToMoveInRevolutions, int homeLimitSwitchPin);
    void moveRelativeInRevolutions(float distanceToMoveInRevolutions);
    void setTargetPositionRelativeInRevolutions(float distanceToMoveInRevolutions);
    void moveToPositionInRevolutions(float absolutePositionToMoveToInRevolutions);
    void setTargetPositionInRevolutions(float absolutePositionToMoveToInRevolutions);
    float getCurrentVelocityInRevolutionsPerSecond();

    void setCurrentPositionInSteps(long currentPositionInSteps);
    long getCurrentPositionInSteps();
    void setSpeedInStepsPerSecond(float speedInStepsPerSecond);
    void setAccelerationInStepsPerSecondPerSecond(float accelerationInStepsPerSecondPerSecond);
    bool moveToHomeInSteps(long directionTowardHome, float speedInStepsPerSecond, long maxDistanceToMoveInSteps, int homeSwitchPin);
    void moveRelativeInSteps(long distanceToMoveInSteps);
    void setTargetPositionRelativeInSteps(long distanceToMoveInSteps);
    void moveToPositionInSteps(long absolutePositionToMoveToInSteps);
    void setTargetPositionInSteps(long absolutePositionToMoveToInSteps);
    void setTargetPositionToStop();
    bool motionComplete();
    float getCurrentVelocityInStepsPerSecond(); 
    bool processMovement(void);


  private:
    //
    // private functions
    //
    void DeterminePeriodOfNextStep();

 
    //
    // private member variables
    //
    int stepPin;
    int directionPin;
    float stepsPerMillimeter;
    float stepsPerRevolution;
    int directionOfMotion;
    long currentPosition_InSteps;
    long targetPosition_InSteps;
    float desiredSpeed_InStepsPerSecond;
    float desiredPeriod_InUSPerStep;
    float acceleration_InStepsPerSecondPerSecond;
    float acceleration_InStepsPerUSPerUS;
    float periodOfSlowestStep_InUS;
    float minimumPeriodForAStoppedMotion;
    float nextStepPeriod_InUS;
    unsigned long lastStepTime_InUS;
    float currentStepPeriod_InUS;
};

// ------------------------------------ End ---------------------------------
#endif

