#include "Motor.h"
#include "Balanced.h"
#include "Follow1.h"

extern Timer2 Timer2;
extern Mpu6050 Mpu6050;
extern Motor Motor;
extern Balanced Balanced;
extern Ultrasonic Ultrasonic;
extern IRLine IR;
extern KalmanFilter kalmanfilter;
Function Function;


void setup() 
{
  Serial.begin(115200);
  delay(10000);
  Motor.Pin_init();
  Ultrasonic.Pin_init();
  Motor.Encoder_init();
  Timer2.init(TIMER);
  Mpu6050.init();
  IR.Pin_init(); 
  Balanced.CalibrateZero(10);
  
  delay(100);
}

void loop()
{
  Function.Follow_Mode1();

  static unsigned long lastPrint = 0;
  if (Function.debug_ready && millis() - lastPrint >= 100) {
    lastPrint = millis();
    Function.debug_ready = false;

    Serial.print("IR L=");
    Serial.print(Function.dbg_l);
    Serial.print(" R=");
    Serial.print(Function.dbg_r);

    Serial.print(" | Dist=");
    Serial.print(Function.dbg_dist);
    Serial.print(" cm | State=");

    switch(Function.dbg_state) {
      case 1: Serial.println("STOP(obstacle)"); break;
      case 2: Serial.println("FWD(line)"); break;
      case 3: Serial.println("LEFT"); break;
      case 4: Serial.println("RIGHT"); break;
      case 5: Serial.println("SEARCH_FWD"); break;
      default: Serial.println("UNKNOWN"); break;
    }
  }
}
