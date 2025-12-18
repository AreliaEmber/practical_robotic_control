#include "Follow1.h"
#include "RGB.h"
#include "Motor.h"

#ifndef SAFE_DISTANCE
#define SAFE_DISTANCE  26.0
#endif
#ifndef Distance_MAX
#define Distance_MAX 70.0
#endif

// === SENSOR-SCHWELLWERTE ===
#define IR_WHITE_MIN      0      // Weiß: 0-100
#define IR_WHITE_MAX      100
#define IR_GRAY_MIN       100    // Grau (Sensor teilweise auf Linie): 100-300
#define IR_GRAY_MAX       300
#define IR_BLACK_MIN      300    // Schwarz: >300

// === LINIENENDEDETEKTIERUNG ===
#define LINE_LOST_TIME    6500   // 3,5 Sekunden kein BLACK erkannt = Linienende
#define FINAL_DISTANCE    10.0   // 25cm nach Linienende fahren
#define ENCODER_TICKS_PER_CM 1  // Encoder-Ticks pro cm (ANPASSEN!)

// === STARTUP-KONFIGURATION ===
#define STARTUP_DELAY     3000

Ultrasonic Ultrasonic;
IRLine IR;
extern Balanced Balanced;

enum RobotState {
    STATE_STARTUP,          // Initialisierungsphase
    STATE_FOLLOW,           // Normales Linienfolgen
    STATE_LINE_LOST,        // Linie verloren erkannt
    STATE_FINAL_STRAIGHT,   // Nach Linienende 25cm geradeaus fahren
    STATE_DONE              // Fertig
};

// === GLOBALE VARIABLEN ===
RobotState current_state = STATE_STARTUP;
unsigned long startup_start_time = 0;
unsigned long line_lost_time = 0;
unsigned long follow_start_time = 0;  // Zeit seit Start des FOLLOW-Modus
unsigned long final_encoder_start = 0;
unsigned long final_distance_encoder_ticks = (unsigned long)(FINAL_DISTANCE * ENCODER_TICKS_PER_CM);
unsigned long encoder_sum_at_line_lost = 0;
bool has_seen_black = false;  // Flag: Hat schwarze Linie bereits gesehen?
const unsigned long MIN_FOLLOW_TIME = 40000;  // Mindestens 40 Sekunden folgen

// ---------- IRLine ----------
void IRLine::Pin_init()
{
  pinMode(IR_LEFT_PIN, INPUT);
  pinMode(IR_RIGHT_PIN, INPUT);
}

void IRLine::Read()
{
  left_raw = analogRead(IR_LEFT_PIN);
  right_raw = analogRead(IR_RIGHT_PIN);
  
  left_on_line = (left_raw > IR_GRAY_MIN);
  right_on_line = (right_raw > IR_GRAY_MIN);
}

bool IRLine::IsOnLine()
{
  return (left_raw > IR_GRAY_MIN) || (right_raw > IR_GRAY_MIN);
}

bool IRLine::IsFullyOnLine()
{
  return (left_raw > IR_BLACK_MIN) && (right_raw > IR_BLACK_MIN);
}

bool IRLine::IsBlackDetected()
{
  return (left_raw > IR_BLACK_MIN) || (right_raw > IR_BLACK_MIN);
}

// ---------- Ultrasonic ----------
void Ultrasonic::Pin_init()
{
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
}

volatile char Ultrasonic::measure_flag = 0;
volatile unsigned long Ultrasonic::measure_prev_time = 0;
volatile unsigned long Ultrasonic::get_distance_prev_time = 0;
volatile double Ultrasonic::distance_value = 0.0;

// ---------- FOLLOW MODE 1 ----------
void Function::Follow_Mode1()
{
    Ultrasonic.Get_Distance();
    IR.Read();
    
    // === DEBUG-AUSGABE ===
    static unsigned long lastDebug = 0;
    if (millis() - lastDebug >= 100) {
        lastDebug = millis();
        dbg_l = IR.left_raw;
        dbg_r = IR.right_raw;
        dbg_dist = Ultrasonic.distance_value;
        debug_ready = true;
    }
    
    // === ENTSCHEIDUNGSLOGIK (alle 80ms) ===
    if (millis() - follow_prev_time >= 80)
    {
        follow_prev_time = millis();
        
        // === STATE MACHINE ===
        switch(current_state) {
            
            case STATE_STARTUP:
                // Initialisierungsphase beim Start
                dbg_state = 100;  // STARTUP
                Balanced.Stop();
                delay(2000);
                current_state = STATE_FOLLOW;
                line_lost_time = 0;
                has_seen_black = false;
                follow_start_time = millis();  // Timer starten
                break;
            
            case STATE_FOLLOW:
                // === BLACK-DETEKTION (IMMER ÜBERPRÜFEN - unabhängig von Verfolgung) ===
                if (IR.IsBlackDetected()) {
                    // BLACK erkannt: Flag setzen und Timer zurücksetzen
                    has_seen_black = true;
                    line_lost_time = 0;
                    dbg_state = 88;  // BLACK DETECTED
                } 
                else if (has_seen_black && line_lost_time == 0) {
                    // BLACK war schon mal da, aber jetzt nicht mehr -> Timer starten
                    line_lost_time = millis();
                    dbg_state = 89;  // TIMER STARTED
                }
                
                // Überprüfe ob 3.5 Sekunden ohne BLACK vergangen sind
                if (has_seen_black && line_lost_time != 0 && 
                    millis() - line_lost_time >= LINE_LOST_TIME &&
                    millis() - follow_start_time >= MIN_FOLLOW_TIME) {
                    // === LINIENENDE ERKANNT (mindestens 30 Sekunden vergangen) ===
                    Balanced.Stop();
                    encoder_sum_at_line_lost = Motor::encoder_count_left_a + Motor::encoder_count_right_a;
                    current_state = STATE_FINAL_STRAIGHT;
                    dbg_state = 50;  // LINE END DETECTED
                    break;
                }
                
                // === NORMALE VERFOLGUNGSLOGIK ===
                if (!IR.IsOnLine()) {
                    // Nicht auf Linie - vorwärts suchen
                    dbg_state = 10;
                    Balanced.Forward(4);
                } 
                else {
                    // Auf Linie - folgen
                    bool L = IR.LeftOnLine();
                    bool R = IR.RightOnLine();
                    
                    if (!L && !R) {
                        Balanced.Forward(4);
                        dbg_state = 2;
                    } 
                    else if (L && !R) {
                        Balanced.Right(12);
                        dbg_state = 3;
                        line_lost_time = 0;
                    } 
                    else if (!L && R) {
                        Balanced.Left(12);
                        dbg_state = 4;
                        line_lost_time = 0;
                    } 
                    else {
                        Balanced.Right(4);
                        dbg_state = 5;
                    }
                }
                break;
                
            case STATE_FINAL_STRAIGHT:
                rgb.blueOn();
                // Nach Linienende 25cm geradeaus fahren
                Balanced.Forward(4);
                
                // Encoder-Ticks seit Linienende zählen
                unsigned long current_encoder_sum = Motor::encoder_count_left_a + Motor::encoder_count_right_a;
                unsigned long encoder_ticks_traveled = current_encoder_sum - encoder_sum_at_line_lost;
                
                dbg_state = 51;  // FINAL STRAIGHT
                
                // Wenn 25cm gefahren: FERTIG
                if (encoder_ticks_traveled >= final_distance_encoder_ticks) {
                    Balanced.Stop();
                    current_state = STATE_DONE;
                    dbg_state = 99;  // DONE
                }
                break;
                
            case STATE_DONE:
                // Fertig - Motor aus
                Balanced.Stop();
                has_seen_black = true;  // Sperren
                dbg_state = 99;
                break;
        }
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