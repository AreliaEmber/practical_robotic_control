#include "Motor.h"
#include "Balanced.h"
#include "Follow1.h"
#include "RGB.h"

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
  delay(6000);
  Motor.Pin_init();
  Ultrasonic.Pin_init();
  Motor.Encoder_init();
  Timer2.init(TIMER);
  Mpu6050.init();
  IR.Pin_init(); 
  rgb.initialize(50);
  rgb.clear();
  rgb.show();
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
    
    // === STATE AUSGABE ===
    switch(Function.dbg_state) {
      case 0: Serial.print("Start_up"); break;
      case 100: Serial.print("Verzögerung"); break;
      case 1: Serial.print("STOP(obstacle)"); break;
      case 2: Serial.print("FWD(line) - Schwarz"); break;
      case 3: Serial.print("LEFT"); break;
      case 4: Serial.print("RIGHT"); break;
      case 5: Serial.print("SEARCH_FWD"); break;
      case 10: Serial.print("Linie suchen"); break;
      case 11: Serial.print("Drehung"); break;
      case 20: Serial.print("Pfad zurück"); break;
      case 50: Serial.print("Linienende"); break;
      case 51: Serial.print("Final Straight"); break;
      case 88: Serial.print("BLACK DETECTED"); break;
      case 89: Serial.print("TIMER STARTED"); break;
      case 99: Serial.print("Nach 25 cm"); break;
      default: Serial.print("UNKNOWN"); break;
    }
    
    // === SENSOR-WERTE ===
    Serial.print(" | L=");
    Serial.print(Function.dbg_l);
    Serial.print(" R=");
    Serial.print(Function.dbg_r);
    Serial.print(" | Dist=");
    Serial.print(Function.dbg_dist);
    Serial.print("cm");
    
    // === ENCODER-TICKS (für STATE_FINAL_STRAIGHT) ===
    if (Function.dbg_state == 51) {
      unsigned long current_sum = Motor::encoder_count_left_a + Motor::encoder_count_right_a;
      unsigned long ticks_traveled = current_sum - encoder_sum_at_line_lost;
      Serial.print(" | Encoder Ticks: ");
      Serial.print(ticks_traveled);
      Serial.print("/");
      Serial.print(final_distance_encoder_ticks);
    }
    
    Serial.println();
  }
}
