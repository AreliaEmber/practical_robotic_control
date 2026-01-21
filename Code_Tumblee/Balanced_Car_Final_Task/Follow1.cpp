#include "Follow1.h"
#include "RGB.h"
#include <queue>
#include <functional>
using namespace std;

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
    STATE_OBSTACLE_FOUND,    // Hindernis erkannt
    STATE_OBSTACLE_AVOID,     // Dreht sich 90 Grad
    STATE_SEARCH       // Sucht nach Linie
};

// === GLOBALE VARIABLEN ===
RobotState current_state = STATE_STARTUP;
bool obstacle_plan_step_finished = false;
unsigned long turn_start_time = 0;
unsigned long startup_start_time = 0;
const unsigned long TURN_180_TIME = 11000;  // Zeit für 180° Drehung (in ms)
const unsigned long FORWARD_5CM_TIME = 5000;  // Zeit für 5cm nach vorne (abgeschätzt erstmal, wird vmtl. etwas anpassung brauchen)
queue<function<void()>> obstacle_handling_plan; // Erstmal hat das keinen Plan, aber alles gut, das geben wir dann später dazu


// hier die lambda funktionen womit wir unsere plan liste befüllen
auto right = []() {
    Balanced.CurveRight(0, 30);  // Nach rechts drehen
    
    if (millis() - turn_start_time >= TURN_180_TIME/2) { // nun 90 grad drehung statt 180 grad
        // Drehung abgeschlossen
        Balanced.Stop();
        obstacle_plan_step_finished = true;
    }
};
auto left = []() {
    Balanced.CurveLeft(0, 30);  // Nach links drehen
    
    if (millis() - turn_start_time >= TURN_180_TIME/2) { // nun 90 grad drehung statt 180 grad
        // Drehung abgeschlossen
        Balanced.Stop();
        obstacle_plan_step_finished = true;
    }
};
auto straight = []() {
    Balanced.Forward(4);  // Langsam nach vorne
    
    if (millis() - turn_start_time >= FORWARD_5CM_TIME) { 
        // Drehung abgeschlossen
        Balanced.Stop();
        obstacle_plan_step_finished = true;
    }
};

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
                Follow_Mode_Startup()
                break;
            
            case STATE_FOLLOW:
                Follow_Mode_Follow()
                break;
                
            case STATE_OBSTACLE_AVOID:
                Follow_Mode_Obstacle()
                break;
            
            case STATE_SEARCH:
                Follow_Mode_Search()
                break;
        }
    }
}

void Function::Follow_Mode_Startup()
{
    // Initialisierungsphase beim Start
    dbg_state = 100;  // STARTUP SUCCESS
    Balanced.Stop();
    delay(2000);
    Enter_Follow_Mode();
}

void Function::Follow_Mode_Follow()
{
    // Normales Linienfolgen
    if (Ultrasonic.distance_value < SAFE_DISTANCE) {
        // HINDERNIS ERKANNT
        Obstacle_Found();
        return;
    } 
    if (!IR.IsOnLine()) {
        // nicht auf Linie, Suchmodus
        dbg_state = 10;  // SEARCH
        Balanced.Forward(4);  // Sehr langsam vorwärts
        return;
    } 
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
void Function::Follow_Mode_Obstacle()
{
    if (obstacle_handling_plan.empty() == 1)
    {
        Enter_Search_Mode();
        dbg_state = 11;  // TURNING COMPLETE
        return;
    }

    obstacle_handling_plan.front()() // wir führen den aktuellen plan-schritt aus

    if (obstacle_plan_step_finished == false) 
    {
        return;
    }

    obstacle_handling_plan.pop() // aktueller plan-schritt ist schon fertig, wir entfernen das vom queue
    obstacle_plan_step_finished = false;

    // entscheidungslogik um weitere pläne zu überlegen passt hier hin

    // ende entscheidungslogik
    

    // kurzer erklärung wie ich das mir aktuell vorstelle:
    // wenn wir erstmal kein hindernis finden, dann gehen wir nach vorne einen schritt
    // im zweiten durchgang würde man dann nach links drehen wenn kein hindernis da ist,
    // dann können wir nämlich schon vorbeifahren.
    // wenn da doch ein hindernis ist, dann drehen wir uns komplett um, gehen nach vorne
    // auf die andere seite vom hindernis und wenn da dann kein hindernis ist, dann drehen
    // wir uns wieder nach rechts and fahren direkt am hindernis vorbei

    // d.h. nach jedem schritt kommen wir wieder hier an und prüfen wie wir weiterfahren sollten
}

void Function::Follow_Mode_Search()
{
    // Nach Hindernis nach der Linie suchen
    if (Ultrasonic.distance_value < SAFE_DISTANCE) {
        // Immer noch Hindernis da - nochmal drehen
        Obstacle_Found();
    } 
    else if (IR.IsOnLine()) {
        // Linie gefunden - zurück zum normalen Folgen
        Enter_Follow_Mode();
        dbg_state = 20;  // LINE FOUND - RESUME FOLLOW
    } 
    else {
        // Noch keine Linie - weiter suchen
        Balanced.Forward(4);  // Langsam vorwärts
        dbg_state = 12;  // SEARCHING
    }
}
// void Function::Follow_Mode_Etc()
// {

// }

void Function::Enter_Follow_Mode()
{
    rgb.greenOn();
    current_state = STATE_FOLLOW;
}
void Function::Obstacle_Found()
{
    Balanced.Stop();
    rgb.blueOn();
    current_state = STATE_OBSTACLE_AVOID;
    turn_start_time = millis();
    dbg_state = 1; // hindernis
    while (obstacle_handling_plan.empty() != 1) {
        obstacle_handling_plan.pop(); // lösche alles was aktuell da drin ist
    }
    obstacle_handling_plan.push(right); //unser erster plan ist immer nach rechts zu drehen
}
void Function::Enter_Search_Mode()
{
    rgb.redOn();
    current_state = STATE_SEARCH;
}

void Ultrasonic::Check()
{
  if (distance_value > SAFE_DISTANCE && distance_value < Distance_MAX) {
    Balanced.Motion_Control(FORWARD);
  } else {
    Balanced.Stop();
  }
}
