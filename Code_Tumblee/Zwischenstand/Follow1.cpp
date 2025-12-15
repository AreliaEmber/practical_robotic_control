#include "Follow1.h"


#ifndef SAFE_DISTANCE
#define SAFE_DISTANCE  20.0  // cm 10~25
#endif

#ifndef Distance_MAX
#define Distance_MAX 70.0    // cm
#endif

Ultrasonic Ultrasonic;
IRLine IR;
extern Balanced Balanced;

// ---------- IRLine ----------
void IRLine::Pin_init()
{
  pinMode(IR_LEFT_PIN, INPUT);
  pinMode(IR_RIGHT_PIN, INPUT);
}

void IRLine::Read()
{
  left_raw  = analogRead(IR_LEFT_PIN);
  right_raw = analogRead(IR_RIGHT_PIN);

  left_on_line  = (left_raw  > IR_THRESHOLD);
  right_on_line = (right_raw > IR_THRESHOLD);
}

void Ultrasonic::Pin_init()
{
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
}

volatile char Ultrasonic::measure_flag = 0;
volatile unsigned long Ultrasonic::measure_prev_time = 0;
volatile unsigned long Ultrasonic::get_distance_prev_time = 0;
volatile double Ultrasonic::distance_value = 0.0;

// ---------- Follow Mode ----------
void Function::Follow_Mode1()
{
  Ultrasonic.Get_Distance();
  IR.Read();

  // alle 50–100ms entscheiden (wie du es schon angefangen hast)
  if (millis() - follow_prev_time >= 80)
  {
    follow_prev_time = millis();

    // 1) Hindernis? => Stop (sicher)
    if (Ultrasonic.distance_value < SAFE_DISTANCE) {
      Balanced.Stop();
      dbg_state = 1; // obstacle stop
    }
    else {
      bool L = IR.LeftOnLine();
      bool R = IR.RightOnLine();

      if (L && R) {
        Balanced.Forward(10);
        dbg_state = 2;
      } else if (L && !R) {
        Balanced.Left(10);
        dbg_state = 3;
      } else if (!L && R) {
        Balanced.Right(10);
        dbg_state = 4;
      } else {
        Balanced.Forward(10);
        dbg_state = 5; // search forward
      }
    }


    // Debugdaten für loop (kein Serial im ISR!)
    dbg_l = IR.LeftRaw();
    dbg_r = IR.RightRaw();
    dbg_dist = Ultrasonic.distance_value;
    debug_ready = true;
  }
}


void Ultrasonic::Check()
{
  if (distance_value > SAFE_DISTANCE && distance_value < Distance_MAX) {
    Balanced.Motion_Control(FORWARD);
  } else {
    Balanced.Stop();
  }
}
