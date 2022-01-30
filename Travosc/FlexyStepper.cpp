

#include "FlexyStepper.h"

//
// direction signal level for "step and direction"
//
#define POSITIVE_DIRECTION LOW
#define NEGATIVE_DIRECTION HIGH


// ---------------------------------------------------------------------------------
//                                  Setup functions 
// ---------------------------------------------------------------------------------


//
// constructor for the stepper class
//
FlexyStepper::FlexyStepper()
{
  //
  // initialize constants
  //
  stepsPerRevolution = 200L;
  stepsPerMillimeter = 25.0;
  directionOfMotion = 0;
  currentPosition_InSteps = 0L;
  targetPosition_InSteps = 0L;
  setSpeedInStepsPerSecond(200);
  setAccelerationInStepsPerSecondPerSecond(200.0);
  currentStepPeriod_InUS = 0.0;
  nextStepPeriod_InUS = 0.0;
}




//
// connect the stepper object to the IO pins
//  Enter:  stepPinNumber = IO pin number for the Step
//          directionPinNumber = IO pin number for the direction bit
//          enablePinNumber = IO pin number for the enable bit (LOW is enabled)
//            set to 0 if enable is not supported
//
void FlexyStepper::connectToPins(int stepPinNumber, int directionPinNumber)
{
  //
  // remember the pin numbers
  //
  stepPin = stepPinNumber;
  directionPin = directionPinNumber;
  
  //
  // configure the IO bits
  //
  pinMode(stepPin, OUTPUT);
  digitalWrite(stepPin, LOW);

  pinMode(directionPin, OUTPUT);
  digitalWrite(directionPin, LOW);
}



// ---------------------------------------------------------------------------------
//                     Public functions with units in millimeters 
// ---------------------------------------------------------------------------------

//
// set the number of steps the motor has per millimeters
//
void FlexyStepper::setStepsPerMillimeter(float motorStepsPerMillimeter)
{
  stepsPerMillimeter = motorStepsPerMillimeter;
}



//
// get the current position of the motor in millimeters, this functions is updated
// while the motor moves
//  Exit:  a signed motor position in millimeters returned
//
float FlexyStepper::getCurrentPositionInMillimeters()
{
  return((float)getCurrentPositionInSteps() / stepsPerMillimeter);
}



//
// set the current position of the motor in millimeters, this does not move the 
// motor
//
void FlexyStepper::setCurrentPositionInMillimeters(
                   float currentPositionInMillimeters)
{
  setCurrentPositionInSteps((long) round(currentPositionInMillimeters * 
                                         stepsPerMillimeter));
}



//
// set the maximum speed, units in millimeters/second, this is the maximum speed  
// reached while accelerating
//  Enter:  speedInMillimetersPerSecond = speed to accelerate up to, units in 
//            millimeters/second
//
void FlexyStepper::setSpeedInMillimetersPerSecond(float speedInMillimetersPerSecond)
{
  setSpeedInStepsPerSecond(speedInMillimetersPerSecond * stepsPerMillimeter);
}



//
// set the rate of acceleration, units in millimeters/second/second
//  Enter:  accelerationInMillimetersPerSecondPerSecond = rate of acceleration,  
//          units in millimeters/second/second
//
void FlexyStepper::setAccelerationInMillimetersPerSecondPerSecond(
                      float accelerationInMillimetersPerSecondPerSecond)
{
    setAccelerationInStepsPerSecondPerSecond(
      accelerationInMillimetersPerSecondPerSecond * stepsPerMillimeter);
}



//
// home the motor by moving until the homing sensor is activated, then set the  
// position to zero, with units in millimeters
//  Enter:  directionTowardHome = 1 to move in a positive direction, -1 to move 
//            in a negative directions 
//          speedInMillimetersPerSecond = speed to accelerate up to while moving 
//            toward home, units in millimeters/second
//          maxDistanceToMoveInMillimeters = unsigned maximum distance to move 
//            toward home before giving up
//          homeSwitchPin = pin number of the home switch, switch should be 
//            configured to go low when at home
//  Exit:   true returned if successful, else false
//
bool FlexyStepper::moveToHomeInMillimeters(long directionTowardHome,  
  float speedInMillimetersPerSecond, long maxDistanceToMoveInMillimeters, 
  int homeLimitSwitchPin)
{
  return(moveToHomeInSteps(directionTowardHome, 
                          speedInMillimetersPerSecond * stepsPerMillimeter, 
                          maxDistanceToMoveInMillimeters * stepsPerMillimeter, 
                          homeLimitSwitchPin));
}



//
// move relative to the current position, units are in millimeters, this function  
// does not return until the move is complete
//  Enter:  distanceToMoveInMillimeters = signed distance to move relative to the  
//          current position in millimeters
//
void FlexyStepper::moveRelativeInMillimeters(float distanceToMoveInMillimeters)
{
  setTargetPositionRelativeInMillimeters(distanceToMoveInMillimeters);
  
  while(!processMovement())
    ;
}



//
// setup a move relative to the current position, units are in millimeters, no   
// motion occurs until processMove() is called
//  Enter:  distanceToMoveInMillimeters = signed distance to move relative to the  
//          current position in millimeters
//
void FlexyStepper::setTargetPositionRelativeInMillimeters(
                     float distanceToMoveInMillimeters)
{
  setTargetPositionRelativeInSteps((long) round(distanceToMoveInMillimeters * 
                                                 stepsPerMillimeter));
}



//
// move to the given absolute position, units are in millimeters, this function 
// does not return until the move is complete
//  Enter:  absolutePositionToMoveToInMillimeters = signed absolute position to  
//          move to in units of millimeters
//
void FlexyStepper::moveToPositionInMillimeters(
                    float absolutePositionToMoveToInMillimeters)
{
  setTargetPositionInMillimeters(absolutePositionToMoveToInMillimeters);
  
  while(!processMovement())
    ;
}



//
// setup a move, units are in millimeters, no motion occurs until processMove() 
// is called
//  Enter:  absolutePositionToMoveToInMillimeters = signed absolute position to  
//          move to in units of millimeters
//
void FlexyStepper::setTargetPositionInMillimeters(
                    float absolutePositionToMoveToInMillimeters)
{
 setTargetPositionInSteps((long) round(absolutePositionToMoveToInMillimeters * 
                                       stepsPerMillimeter));
}



//
// Get the current velocity of the motor in millimeters/second.  This functions is 
// updated while it accelerates up and down in speed.  This is not the desired  
// speed, but the speed the motor should be moving at the time the function is   
// called.  This is a signed value and is negative when the motor is moving 
// backwards.  Note: This speed will be incorrect if the desired velocity is set 
// faster than this library can generate steps, or if the load on the motor is too 
// great for the amount of torque that it can generate.
//  Exit:  velocity speed in steps per second returned, signed
//
float FlexyStepper::getCurrentVelocityInMillimetersPerSecond()
{
  return(getCurrentVelocityInStepsPerSecond() / stepsPerMillimeter);
}



// ---------------------------------------------------------------------------------
//                     Public functions with units in revolutions 
// ---------------------------------------------------------------------------------

//
// set the number of steps the motor has per revolution
//
void FlexyStepper::setStepsPerRevolution(float motorStepPerRevolution)
{
  stepsPerRevolution = motorStepPerRevolution;
}



//
// get the current position of the motor in revolutions, this functions is updated
// while the motor moves
//  Exit:  a signed motor position in revolutions returned
//
float FlexyStepper::getCurrentPositionInRevolutions()
{
  return((float)getCurrentPositionInSteps() / stepsPerRevolution);
}



//
// set the current position of the motor in revolutions, this does not move the 
// motor
//
void FlexyStepper::setCurrentPositionInRevolutions(
                     float currentPositionInRevolutions)
{
  setCurrentPositionInSteps((long) round(currentPositionInRevolutions * 
                                         stepsPerRevolution));
}



//
// set the maximum speed, units in revolutions/second, this is the maximum speed  
// reached while accelerating
//  Enter:  speedInRevolutionsPerSecond = speed to accelerate up to, units in 
//            revolutions/second
//
void FlexyStepper::setSpeedInRevolutionsPerSecond(float speedInRevolutionsPerSecond)
{
  setSpeedInStepsPerSecond(speedInRevolutionsPerSecond * stepsPerRevolution);
}



//
// set the rate of acceleration, units in revolutions/second/second
//  Enter:  accelerationInRevolutionsPerSecondPerSecond = rate of acceleration,  
//          units in revolutions/second/second
//
void FlexyStepper::setAccelerationInRevolutionsPerSecondPerSecond(
       float accelerationInRevolutionsPerSecondPerSecond)
{
    setAccelerationInStepsPerSecondPerSecond(
      accelerationInRevolutionsPerSecondPerSecond * stepsPerRevolution);
}



//
// home the motor by moving until the homing sensor is activated, then set the 
//  position to zero, with units in revolutions
//  Enter:  directionTowardHome = 1 to move in a positive direction, -1 to move in 
//            a negative directions 
//          speedInRevolutionsPerSecond = speed to accelerate up to while moving 
//            toward home, units in revolutions/second
//          maxDistanceToMoveInRevolutions = unsigned maximum distance to move 
//            toward home before giving up
//          homeSwitchPin = pin number of the home switch, switch should be 
//            configured to go low when at home
//  Exit:   true returned if successful, else false
//
bool FlexyStepper::moveToHomeInRevolutions(long directionTowardHome,  
  float speedInRevolutionsPerSecond, long maxDistanceToMoveInRevolutions, 
  int homeLimitSwitchPin)
{
  return(moveToHomeInSteps(directionTowardHome, 
                          speedInRevolutionsPerSecond * stepsPerRevolution, 
                          maxDistanceToMoveInRevolutions * stepsPerRevolution, 
                          homeLimitSwitchPin));
}



//
// move relative to the current position, units are in revolutions, this function  
// does not return until the move is complete
//  Enter:  distanceToMoveInRevolutions = signed distance to move relative to the  
//          current position in revolutions
//
void FlexyStepper::moveRelativeInRevolutions(float distanceToMoveInRevolutions)
{
  setTargetPositionRelativeInRevolutions(distanceToMoveInRevolutions);
  
  while(!processMovement())
    ;
}



//
// setup a move relative to the current position, units are in revolutions, no   
// motion occurs until processMove() is called
//  Enter:  distanceToMoveInRevolutions = signed distance to move relative to the  
//            currentposition in revolutions
//
void FlexyStepper::setTargetPositionRelativeInRevolutions(
                     float distanceToMoveInRevolutions)
{
  setTargetPositionRelativeInSteps((long) round(distanceToMoveInRevolutions * 
                                                stepsPerRevolution));
}



//
// move to the given absolute position, units are in revolutions, this function 
// does not return until the move is complete
//  Enter:  absolutePositionToMoveToInRevolutions = signed absolute position to 
//            move to in units of revolutions
//
void FlexyStepper::moveToPositionInRevolutions(
                    float absolutePositionToMoveToInRevolutions)
{
  setTargetPositionInRevolutions(absolutePositionToMoveToInRevolutions);
  
  while(!processMovement())
    ;
}



//
// setup a move, units are in revolutions, no motion occurs until processMove() 
// is called
//  Enter:  absolutePositionToMoveToInRevolutions = signed absolute position to  
//          move to in units of revolutions
//
void FlexyStepper::setTargetPositionInRevolutions(
       float absolutePositionToMoveToInRevolutions)
{
 setTargetPositionInSteps((long) round(absolutePositionToMoveToInRevolutions * 
                                        stepsPerRevolution));
}



//
// Get the current velocity of the motor in revolutions/second.  This functions is 
// updated while it accelerates up and down in speed.  This is not the desired  
// speed, but the speed the motor should be moving at the time the function is   
// called.  This is a signed value and is negative when the motor is moving 
// backwards.  Note: This speed will be incorrect if the desired velocity is set 
// faster than this library can generate steps, or if the load on the motor is too 
// great for the amount of torque that it can generate.
//  Exit:  velocity speed in steps per second returned, signed
//
float FlexyStepper::getCurrentVelocityInRevolutionsPerSecond()
{
  return(getCurrentVelocityInStepsPerSecond() / stepsPerRevolution);
}


// ---------------------------------------------------------------------------------
//                        Public functions with units in steps 
// ---------------------------------------------------------------------------------

//
// set the current position of the motor in steps, this does not move the motor
// Note: This function should only be called when the motor is stopped
//    Enter:  currentPositionInSteps = the new position of the motor in steps
//
void FlexyStepper::setCurrentPositionInSteps(long currentPositionInSteps)
{
  currentPosition_InSteps = currentPositionInSteps;
}



//
// get the current position of the motor in steps, this functions is updated
// while the motor moves
//  Exit:  a signed motor position in steps returned
//
long FlexyStepper::getCurrentPositionInSteps()
{
  return(currentPosition_InSteps);
}



//
// set the maximum speed, units in steps/second, this is the maximum speed reached  
// while accelerating
//  Enter:  speedInStepsPerSecond = speed to accelerate up to, units in steps/second
//
void FlexyStepper::setSpeedInStepsPerSecond(float speedInStepsPerSecond)
{
  desiredSpeed_InStepsPerSecond = speedInStepsPerSecond;
  desiredPeriod_InUSPerStep = 1000000.0 / desiredSpeed_InStepsPerSecond;
}



//
// set the rate of acceleration, units in steps/second/second
//  Enter:  accelerationInStepsPerSecondPerSecond = rate of acceleration, units in 
//          steps/second/second
//
void FlexyStepper::setAccelerationInStepsPerSecondPerSecond(
                     float accelerationInStepsPerSecondPerSecond)
{
  acceleration_InStepsPerSecondPerSecond = accelerationInStepsPerSecondPerSecond;
  acceleration_InStepsPerUSPerUS = acceleration_InStepsPerSecondPerSecond / 1E12;

  periodOfSlowestStep_InUS = 
      1000000.0 / sqrt(2.0 * acceleration_InStepsPerSecondPerSecond);
  minimumPeriodForAStoppedMotion = periodOfSlowestStep_InUS / 2.8;
}



//
// home the motor by moving until the homing sensor is activated, then set the 
// position to zero with units in steps
//  Enter:  directionTowardHome = 1 to move in a positive direction, -1 to move in 
//            a negative directions 
//          speedInStepsPerSecond = speed to accelerate up to while moving toward 
//            home, units in steps/second
//          maxDistanceToMoveInSteps = unsigned maximum distance to move toward 
//            home before giving up
//          homeSwitchPin = pin number of the home switch, switch should be 
//            configured to go low when at home
//  Exit:   true returned if successful, else false
//
bool FlexyStepper::moveToHomeInSteps(long directionTowardHome,  
  float speedInStepsPerSecond, long maxDistanceToMoveInSteps, 
  int homeLimitSwitchPin)
{
  float originalDesiredSpeed_InStepsPerSecond;
  bool limitSwitchFlag;
  
  
  //
  // setup the home switch input pin
  //
  pullUpDnControl(homeLimitSwitchPin, PUD_DOWN);
  
  
  //
  // remember the current speed setting
  //
  originalDesiredSpeed_InStepsPerSecond = desiredSpeed_InStepsPerSecond; 
 
 
  //
  // if the home switch is not already set, move toward it
  //
  if (digitalRead(homeLimitSwitchPin) == HIGH)
  {
    //
    // move toward the home switch
    //
    setSpeedInStepsPerSecond(speedInStepsPerSecond);
    setTargetPositionRelativeInSteps(maxDistanceToMoveInSteps * directionTowardHome);
    limitSwitchFlag = false;
    while(!processMovement())
    {
      if (digitalRead(homeLimitSwitchPin) == LOW)
      {
        limitSwitchFlag = true;
        directionOfMotion = 0;
        break;
      }
    }
    
    //
    // check if switch never detected
    //
    if (limitSwitchFlag == false)
      return(false);
  }
  delay(25);
  

  //
  // the switch has been detected, now move away from the switch
  //
  setTargetPositionRelativeInSteps(maxDistanceToMoveInSteps * 
                                   directionTowardHome * -1);
  limitSwitchFlag = false;
  while(!processMovement())
  {
    if (digitalRead(homeLimitSwitchPin) == HIGH)
    {
      limitSwitchFlag = true;
      directionOfMotion = 0;
      break;
    }
  }
  delay(25);
  
  //
  // check if switch never detected
  //
  if (limitSwitchFlag == false)
    return(false);


  //
  // have now moved off the switch, move toward it again but slower
  //
  setSpeedInStepsPerSecond(speedInStepsPerSecond/8);
  setTargetPositionRelativeInSteps(maxDistanceToMoveInSteps * directionTowardHome);
  limitSwitchFlag = false;
  while(!processMovement())
  {
    if (digitalRead(homeLimitSwitchPin) == LOW)
    {
      limitSwitchFlag = true;
      directionOfMotion = 0;
      break;
    }
  }
  delay(25);
  
  //
  // check if switch never detected
  //
  if (limitSwitchFlag == false)
    return(false);


  //
  // successfully homed, set the current position to 0
  //
  setCurrentPositionInSteps(0L);    

  //
  // restore original velocity
  //
  setSpeedInStepsPerSecond(originalDesiredSpeed_InStepsPerSecond);
  return(true);
}



//
// move relative to the current position, units are in steps, this function does 
// not return until the move is complete
//  Enter:  distanceToMoveInSteps = signed distance to move relative to the current  
//          position in steps
//
void FlexyStepper::moveRelativeInSteps(long distanceToMoveInSteps)
{
  setTargetPositionRelativeInSteps(distanceToMoveInSteps);
  
  while(!processMovement())
    ;
}



//
// setup a move relative to the current position, units are in steps, no motion  
// occurs until processMove() is called
//  Enter:  distanceToMoveInSteps = signed distance to move relative to the current  
//            positionin steps
//
void FlexyStepper::setTargetPositionRelativeInSteps(long distanceToMoveInSteps)
{
  setTargetPositionInSteps(currentPosition_InSteps + distanceToMoveInSteps);
}



//
// move to the given absolute position, units are in steps, this function does not 
// return until the move is complete
//  Enter:  absolutePositionToMoveToInSteps = signed absolute position to move to  
//            in unitsof steps
//
void FlexyStepper::moveToPositionInSteps(long absolutePositionToMoveToInSteps)
{
  setTargetPositionInSteps(absolutePositionToMoveToInSteps);
  
  while(!processMovement())
    ;
}



//
// setup a move, units are in steps, no motion occurs until processMove() is called
//  Enter:  absolutePositionToMoveToInSteps = signed absolute position to move to  
//            in units of steps
//
void FlexyStepper::setTargetPositionInSteps(long absolutePositionToMoveToInSteps)
{
  targetPosition_InSteps = absolutePositionToMoveToInSteps;
}



//
// setup a "Stop" to begin the process of decelerating from the current velocity  
// to zero, decelerating requires calls to processMove() until the move is complete
// Note: This function can be used to stop a motion initiated in units of steps 
// or revolutions
//
void FlexyStepper::setTargetPositionToStop()
{
  long decelerationDistance_InSteps;
  

  if(targetPosition_InSteps!= currentPosition_InSteps){
  
    //
    // move the target position so that the motor will begin deceleration now
    //
      decelerationDistance_InSteps = (long)round(
          5E11 / (acceleration_InStepsPerSecondPerSecond * currentStepPeriod_InUS *
              currentStepPeriod_InUS));

      if (directionOfMotion > 0)
          setTargetPositionInSteps(currentPosition_InSteps + decelerationDistance_InSteps);
      else
          setTargetPositionInSteps(currentPosition_InSteps - decelerationDistance_InSteps);
 
  }

}



//
// if it is time, move one step
//  Exit:  true returned if movement complete, false returned not a final target 
//           position yet
//
bool FlexyStepper::processMovement(void)
{ 
  unsigned long currentTime_InUS;
  unsigned long periodSinceLastStep_InUS;
  long distanceToTarget_Signed;


  //
  // check if currently stopped
  //
  if (directionOfMotion == 0)
  {
    distanceToTarget_Signed = targetPosition_InSteps - currentPosition_InSteps;

    //
    // check if target position in a positive direction
    //
    if (distanceToTarget_Signed > 0)
    {
      directionOfMotion = 1;
      digitalWrite(directionPin, POSITIVE_DIRECTION);
      nextStepPeriod_InUS = periodOfSlowestStep_InUS;
      lastStepTime_InUS = micros(); 
      return(false);
    }
    
    //
    // check if target position in a negative direction
    //
    else if (distanceToTarget_Signed < 0)
    {
      directionOfMotion = -1;
      digitalWrite(directionPin, NEGATIVE_DIRECTION);
      nextStepPeriod_InUS = periodOfSlowestStep_InUS;
      lastStepTime_InUS = micros(); 
      return(false);
    }
    
    else
      return(true);
  }
   
    
  //
  // determine how much time has elapsed since the last step (Note 1: this method   
  // works even if the time has wrapped. Note 2: all variables must be unsigned)
  //
  currentTime_InUS = micros();
  periodSinceLastStep_InUS = currentTime_InUS - lastStepTime_InUS;


  //
  // if it is not time for the next step, return
  //
  if (periodSinceLastStep_InUS < (unsigned long) nextStepPeriod_InUS)
    return(false);
  

  //
  // execute the step on the rising edge
  //
  digitalWrite(stepPin, HIGH);
  
  //
  // this delay almost nothing because there's so much code between rising & 
  // falling edges
  //
  delayMicroseconds(2);       
  
  
  //
  // update the current position and speed
  //
  currentPosition_InSteps += directionOfMotion;
  currentStepPeriod_InUS = nextStepPeriod_InUS;


  //
  // remember the time that this step occured
  //
  lastStepTime_InUS = currentTime_InUS;
 
 
  //
  // figure out how long before the next step
  //
  DeterminePeriodOfNextStep();
 

  //
  // return the step line low
  //
  digitalWrite(stepPin, LOW);


  //
  // check if the move has reached its final target position, return true if all 
  // done
  //
  if (currentPosition_InSteps == targetPosition_InSteps)
  {
    //
    // at final position, make sure the motor is not going too fast
    //
    if (nextStepPeriod_InUS >= minimumPeriodForAStoppedMotion) 
    {
      currentStepPeriod_InUS = 0.0;
      nextStepPeriod_InUS = 0.0;
      directionOfMotion = 0;
      return(true);
    }
  }
    
  return(false);
}



//
// Get the current velocity of the motor in steps/second.  This functions is 
// updated while it accelerates up and down in speed.  This is not the desired  
// speed, but the speed the motor should be moving at the time the function is  
// called.  This is a signed value and is negative when the motor is moving 
// backwards.  Note: This speed will be incorrect if the desired velocity is set 
// faster than this library can generate steps, or if the load on the motor is too 
// great for the amount of torque that it can generate.
//  Exit:  velocity speed in steps per second returned, signed
//
float FlexyStepper::getCurrentVelocityInStepsPerSecond()
{
  if (currentStepPeriod_InUS == 0.0)
    return(0);
  else
  {
    if (directionOfMotion > 0)
      return(1000000.0 / currentStepPeriod_InUS);
    else
      return(-1000000.0 / currentStepPeriod_InUS);
  }
}



//
// check if the motor has competed its move to the target position
//  Exit:  true returned if the stepper is at the target position
//
bool FlexyStepper::motionComplete()
{
  if ((directionOfMotion == 0) && 
      (currentPosition_InSteps == targetPosition_InSteps))
    return(true);
  else
    return(false);
}



//
// determine the period for the next step, either speed up a little, slow down a  
// little or go the same speed
//
void FlexyStepper::DeterminePeriodOfNextStep()
{
  long distanceToTarget_Signed;
  long distanceToTarget_Unsigned;
  float currentSpeed_InStepsPerSecond;
  long decelerationDistance_InSteps;
  float currentStepPeriodSquared;
  bool speedUpFlag = false;
  bool slowDownFlag = false;
  bool targetInPositiveDirectionFlag = false;
  bool targetInNegativeDirectionFlag = false;

  
  //
  // determine the distance to the target position
  //
  distanceToTarget_Signed = targetPosition_InSteps - currentPosition_InSteps;
  if (distanceToTarget_Signed >= 0L)
  {
    distanceToTarget_Unsigned = distanceToTarget_Signed;
    targetInPositiveDirectionFlag = true;
  }
  else
  {
    distanceToTarget_Unsigned = -distanceToTarget_Signed;
    targetInNegativeDirectionFlag = true;
  }


  //
  // determine the number of steps needed to go from the current speed down to a 
  // velocity of 0, Steps = Velocity^2 / (2 * Acceleration)
  //
  currentStepPeriodSquared = currentStepPeriod_InUS * currentStepPeriod_InUS;
  decelerationDistance_InSteps = (long) round(
    5E11 / (acceleration_InStepsPerSecondPerSecond * currentStepPeriodSquared));
  
  
  //
  // check if: Moving in a positive direction & Moving toward the target
  //    (directionOfMotion == 1) && (distanceToTarget_Signed > 0)
  //
  if ((directionOfMotion == 1) && (targetInPositiveDirectionFlag))
  {
    //
    // check if need to start slowing down as we reach the target, or if we 
    // need to slow down because we are going too fast
    //
    if ((distanceToTarget_Unsigned < decelerationDistance_InSteps) || 
        (nextStepPeriod_InUS < desiredPeriod_InUSPerStep))
      slowDownFlag = true;
    else 
      speedUpFlag = true;
   }


  //
  // check if: Moving in a positive direction & Moving away from the target
  //    (directionOfMotion == 1) && (distanceToTarget_Signed < 0)
  //
  else if ((directionOfMotion == 1) && (targetInNegativeDirectionFlag))
  {
    //
    // need to slow down, then reverse direction
    //
    if (currentStepPeriod_InUS < periodOfSlowestStep_InUS)
    {
      slowDownFlag = true;
    }
    else
    {
      directionOfMotion = -1;
      digitalWrite(directionPin, NEGATIVE_DIRECTION);
    }
  }


  //
  // check if: Moving in a negative direction & Moving toward the target
  //    (directionOfMotion == -1) && (distanceToTarget_Signed < 0)
  //
  else if ((directionOfMotion == -1) && (targetInNegativeDirectionFlag))
  {
    //
    // check if need to start slowing down as we reach the target, or if we 
    // need to slow down because we are going too fast
    //
    if ((distanceToTarget_Unsigned < decelerationDistance_InSteps) || 
        (nextStepPeriod_InUS < desiredPeriod_InUSPerStep))
      slowDownFlag = true;
    else 
      speedUpFlag = true;
   }


  //
  // check if: Moving in a negative direction & Moving away from the target
  //    (directionOfMotion == -1) && (distanceToTarget_Signed > 0)
  //
  else if ((directionOfMotion == -1) && (targetInPositiveDirectionFlag))
  {
    //
    // need to slow down, then reverse direction
    //
    if (currentStepPeriod_InUS < periodOfSlowestStep_InUS)
    {
      slowDownFlag = true;
    }
    else
    {
      directionOfMotion = 1;
      digitalWrite(directionPin, POSITIVE_DIRECTION);
    }
  }


  //
  // check if accelerating
  //
  if (speedUpFlag)
  {
    //
    // StepPeriod = StepPeriod(1 - a * StepPeriod^2)
    //
    nextStepPeriod_InUS = currentStepPeriod_InUS - acceleration_InStepsPerUSPerUS * 
      currentStepPeriodSquared * currentStepPeriod_InUS;

    if (nextStepPeriod_InUS < desiredPeriod_InUSPerStep)
      nextStepPeriod_InUS = desiredPeriod_InUSPerStep;
  }

  
  //
  // check if decelerating
  //
  if (slowDownFlag)
  {
    //
    // StepPeriod = StepPeriod(1 + a * StepPeriod^2)
    //
    nextStepPeriod_InUS = currentStepPeriod_InUS + acceleration_InStepsPerUSPerUS * 
      currentStepPeriodSquared * currentStepPeriod_InUS;

    if (nextStepPeriod_InUS > periodOfSlowestStep_InUS)
      nextStepPeriod_InUS = periodOfSlowestStep_InUS;
  }
}


// -------------------------------------- End --------------------------------------

