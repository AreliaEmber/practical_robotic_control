#ifndef MODE_H
#define MODE_H

// Function modes
enum FunctionMode
{
  IDLE,
  TASK1,        // Neue: Linienverfolgung Task1
  TASK2,        // Neue: Linienverfolgung Task2
  IRREMOTE,
  OBSTACLE,
  FOLLOW,
  BLUETOOTH,
  FOLLOW2
};

// Motion modes bleiben gleich
enum MotionMode
{
  FORWARD,
  BACKWARD,
  TURNLEFT,
  TURNRIGHT,
  STANDBY,
  STOP,
  START
};

extern FunctionMode function_mode;
extern MotionMode motion_mode;

FunctionMode function_mode = IDLE;
MotionMode motion_mode = STOP;

#endif // MODE_H