#ifndef _FOLLOW1_H_
#define _FOLLOW1_H_

#include <Arduino.h>
#include "Balanced.h"


class Ultrasonic 
{
  public:   
          void Pin_init();
          void Get_Distance();
          void Check();
          static void Distance_Measure();
          static void PrintStatus();

          static volatile char measure_flag;
          static volatile unsigned long measure_prev_time;
          static volatile double distance_value;
          static volatile unsigned long get_distance_prev_time;


          
  private:
          #define ECHO_PIN 17
          #define TRIG_PIN 11 
          #define Distance_MIN 3
          #define Distance_MAX 35 
          #define DISTANCE_JUDAGEMENT (distance_value > Distance_MIN && distance_value < Distance_MAX)

};

class IRLine
{
  public:
          void Pin_init();
          void Read();            
          int  LeftRaw()  const { return left_raw; }
          int  RightRaw() const { return right_raw; }
          bool LeftOnLine()  const { return left_on_line; }
          bool RightOnLine() const { return right_on_line; }

  private:
          #define IR_LEFT_PIN  A0
          #define IR_RIGHT_PIN A1
          #define IR_THRESHOLD 512
          int left_raw = 0;
          int right_raw = 0;
          bool left_on_line = false;
          bool right_on_line = false;
};


class Function
{
  public:
         void Follow_Mode1();

  public:
          int follow_flag;
          unsigned long follow_prev_time;
          // ---- Debug (für Serial im loop) ----
          volatile bool debug_ready = false;
          volatile int dbg_l = 0;
          volatile int dbg_r = 0;
          volatile double dbg_dist = 0;
          volatile uint8_t dbg_state = 0;  // 1=STOP(Obstacle),2=FWD(line),3=LEFT,4=RIGHT,5=SEARCH
};

enum FOLLW_MOTION
{
  FOLLOW_LEFT = 1,
  FOLLOW_RIGHT,
  FOLLOW_BACK,
};



#endif
