#include "Follow1.h"
#include "RGB.h"

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

// === STARTUP-KONFIGURATION ===
#define STARTUP_DELAY     3000   // 2 Sekunden Verzögerung vor Start

Ultrasonic Ultrasonic;
IRLine IR;
extern Balanced Balanced;

enum RobotState {
    STATE_STARTUP,     // Initialisierungsphase
    STATE_FOLLOW,      // Normales Linienfolgen
    STATE_OBSTACLE,    // Hindernis erkannt
    STATE_TURNING,     // Dreht sich 180 Grad
    STATE_SEARCH       // Sucht nach Linie
};

// === GLOBALE VARIABLEN ===
RobotState current_state = STATE_STARTUP;
unsigned long turn_start_time = 0;
unsigned long startup_start_time = 0;
const unsigned long TURN_180_TIME = 10000;  // Zeit für 180° Drehung (in ms)

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
                dbg_state = 100;  // STARTUP SUCCESS
                Balanced.Stop();
                delay(2000);
                current_state = STATE_FOLLOW;
            
                break;
            
            case STATE_FOLLOW:
                // Normales Linienfolgen
                if (Ultrasonic.distance_value < SAFE_DISTANCE) {
                    // HINDERNIS ERKANNT
                    rgb.blueOn();
                    Balanced.Stop();
                    current_state = STATE_TURNING;
                    turn_start_time = millis();
                    dbg_state = 1;  // STOP - Hindernis
                } 
                else if (!IR.IsOnLine()) {
                    // nicht auf Linie, Suchmodus
                    dbg_state = 10;  // SEARCH
                    Balanced.Forward(4);  // Sehr langsam vorwärts
                } 
                else {
                    // LINIE GEFUNDEN: TRACKING
                    bool L = IR.LeftOnLine();
                    bool R = IR.RightOnLine();
                    
                    if (!L && !R) {
                        // Beide perfekt neben der Linie: Geradeaus
                        Balanced.Forward(4);
                        dbg_state = 2;
                    } 
                    else if (L && !R) {
                        // Nur links: Rechts korrigieren
                        Balanced.Right(12);
                        dbg_state = 3;
                    } 
                    else if (!L && R) {
                        // Nur rechts: Links korrigieren
                        Balanced.Left(12);
                        dbg_state = 4;
                    } 
                    else {
                        // Beide auf Linie: Nach rechts korrigieren
                        Balanced.Right(4);
                        dbg_state = 5;
                    }
                }
                break;
                
            case STATE_TURNING:
                // 180 Grad Drehung
                Balanced.CurveRight(0, 30);  // Nach rechts drehen
                
                if (millis() - turn_start_time >= TURN_180_TIME) {
                    // 180° Drehung abgeschlossen
                    Balanced.Stop();
                    current_state = STATE_SEARCH;
                    dbg_state = 11;  // TURNING COMPLETE
                }
                break;
                
            case STATE_SEARCH:
                rgb.off();
                // Nach Hindernis nach der Linie suchen
                if (Ultrasonic.distance_value < SAFE_DISTANCE) {
                    // Immer noch Hindernis da - nochmal drehen
                    Balanced.Stop();
                    current_state = STATE_TURNING;
                    turn_start_time = millis();
                    dbg_state = 1;
                } 
                else if (IR.IsOnLine()) {
                    // Linie gefunden - zurück zum normalen Folgen
                    current_state = STATE_FOLLOW;
                    dbg_state = 20;  // LINE FOUND - RESUME FOLLOW
                } 
                else {
                    // Noch keine Linie - weiter suchen
                    Balanced.Forward(4);  // Langsam vorwärts
                    dbg_state = 12;  // SEARCHING
                }
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
