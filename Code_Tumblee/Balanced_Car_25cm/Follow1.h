#ifndef _FOLLOW1_H_
#define _FOLLOW1_H_

#include <Arduino.h>
#include "Balanced.h"

extern unsigned long encoder_sum_at_line_lost;
extern unsigned long final_distance_encoder_ticks;

// === ULTRASONIC SENSOR KLASSE ===
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

// === IR SENSOR KLASSE ===
class IRLine
{
public:
  void Pin_init();
  void Read();            
  
  // Getter
  int LeftRaw() const { return left_raw; }
  int RightRaw() const { return right_raw; }
  bool LeftOnLine() const { return left_on_line; }
  bool RightOnLine() const { return right_on_line; }
  bool IsOnLine();        // Mindestens ein Sensor sieht Linie
  bool IsFullyOnLine();   // Beide Sensoren sehen Linie
  bool IsBlackDetected();
  int left_raw = 0;
  int right_raw = 0;
  
  
  
private:
  #define IR_LEFT_PIN  A0
  #define IR_RIGHT_PIN A1
  #define IR_THRESHOLD 750
  bool left_on_line = false;
  bool right_on_line = false;

};

// === FUNCTION KLASSE (HAUPTLOGIK) ===
class Function
{
public:
  void Follow_Mode1();
  
  // Debug-Variablen (NUR EINMAL!)
  volatile int dbg_l = 0;
  volatile int dbg_r = 0;
  volatile double dbg_dist = 0;
  volatile int dbg_state = 0;
  volatile bool debug_ready = false;
  
  // Follow-Logik
  int follow_flag = 0;
  unsigned long follow_prev_time = 0;
  unsigned long search_start_time = 0;
  char search_direction = 0;  // 0=forward, 1=right, 2=left
};

// === MOTION ENUM ===
enum FOLLW_MOTION
{
  FOLLOW_LEFT = 1,
  FOLLOW_RIGHT,
  FOLLOW_BACK,
};

#endif // _FOLLOW1_H_
