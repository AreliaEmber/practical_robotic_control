#include "Follow1.h"
#include "actuators/RGB.h"
#include "actuators/Motor.h"

using namespace std;

#ifndef SAFE_DISTANCE
#define SAFE_DISTANCE  26.0
#endif
#ifndef Distance_MAX
#define Distance_MAX 70.0
#endif

// === SENSOR-SCHWELLWERTE ===
#define IR_WHITE_MIN      0      // Weiß: 0-100  // hab ich geändert, da der winkel von den sensoren nun anders ist
#define IR_WHITE_MAX      700
#define IR_GRAY_MIN       700    // Grau (Sensor teilweise auf Linie): 100-300
#define IR_GRAY_MAX       800
#define IR_BLACK_MIN      800    // Schwarz: >300

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
    STATE_STARTUP,     // Initialisierungsphase
    STATE_FOLLOW,      // Normales Linienfolgen
    STATE_OBSTACLE_FOUND,    // Hindernis erkannt
    STATE_OBSTACLE_AVOID,     // Dreht sich 90 Grad
    STATE_SEARCH       // Sucht nach Linie
};

// === GLOBALE VARIABLEN ===
RobotState current_state = STATE_STARTUP;
ObstacleState obstacle_handling_plan[16] = {
    OBS_NONE, OBS_NONE, OBS_NONE, OBS_NONE,
    OBS_NONE, OBS_NONE, OBS_NONE, OBS_NONE,
    OBS_NONE, OBS_NONE, OBS_NONE, OBS_NONE,
    OBS_NONE, OBS_NONE, OBS_NONE, OBS_NONE
};
unsigned long current_obstacle_step = 0;
bool obstacle_plan_step_finished = false;
unsigned long turn_start_time = 0;
unsigned long startup_start_time = 0;
unsigned long line_lost_time = 0;
unsigned long follow_start_time = 0;  // Zeit seit Start des FOLLOW-Modus
unsigned long final_encoder_start = 0;
unsigned long final_distance_encoder_ticks = (unsigned long)(FINAL_DISTANCE * ENCODER_TICKS_PER_CM);
unsigned long encoder_sum_at_line_lost = 0;
bool has_seen_black = false;  // Flag: Hat schwarze Linie bereits gesehen?
const unsigned long MIN_FOLLOW_TIME = 40000;  // Mindestens 40 Sekunden folgen
const unsigned long TURN_180_TIME = 6000;  // Zeit für 180° Drehung (in ms)
const unsigned long FORWARD_5CM_TIME = 2500;  // Zeit für 5cm nach vorne (abgeschätzt erstmal, wird vmtl. etwas anpassung brauchen)
const unsigned long FORWARD_SPEED = 12;
const unsigned long RIGHT_SPEED = 36;
const unsigned long LEFT_SPEED = 36;

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
                Follow_Mode_Startup();
                break;
            
            case STATE_FOLLOW:
                Follow_Mode_Follow();
                break;
                
            case STATE_OBSTACLE_AVOID:
                Follow_Mode_Obstacle();
                break;
            
            case STATE_SEARCH:
                Follow_Mode_Search();
                break;
            case STATE_OBSTACLE_FOUND:
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
        Balanced.Forward(FORWARD_SPEED);  // Sehr langsam vorwärts
        return;
    } 
    // LINIE GEFUNDEN: TRACKING
    bool L = IR.LeftOnLine();
    bool R = IR.RightOnLine();
    
    if (L && R) {
        // Beide perfekt neben der Linie: Geradeaus
        Balanced.Forward(FORWARD_SPEED);
        dbg_state = 2;
    } 
    else if (!L && R) {
        // Nur links: Rechts korrigieren
        Balanced.Right(RIGHT_SPEED);
        dbg_state = 3;
    } 
    else if (L && !R) {
        // Nur rechts: Links korrigieren
        Balanced.Left(LEFT_SPEED);
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
    switch (obstacle_handling_plan[current_obstacle_step]) {
        case OBS_FORWARD:
            Serial.println("forward");
            break;
        
        case OBS_LEFT:
            Serial.println("left");
            break;
            
        case OBS_RIGHT:
            Serial.println("right");
            break;
        
        case OBS_NONE:
            Serial.println("none");
            break;
    }

    if (obstacle_handling_plan[current_obstacle_step] == OBS_NONE)
    {
        Enter_Search_Mode();
        dbg_state = 11;  // TURNING COMPLETE
        return;
    }

    Execute_Obstacle_Step(obstacle_handling_plan[current_obstacle_step]); // wir führen den aktuellen plan-schritt aus

    if (obstacle_plan_step_finished == false) // müssen weiter das gleiche ausführen
    {
        return;
    }

    current_obstacle_step++; // aktueller plan-schritt ist schon fertig, wir iterieren
    turn_start_time = millis();
    obstacle_plan_step_finished = false;

    if (obstacle_handling_plan[current_obstacle_step] != OBS_NONE) // es gibt noch weitere pläne die wir ausführen können
    {
        return;
    }

    // entscheidungslogik aktuell einfach gehardcodet. why not

    if (current_obstacle_step == 1) // hardcoden von der zeit her weil das einfacher geht
    {
        if (Ultrasonic.distance_value < SAFE_DISTANCE/2) // es gibt auch hier ein hindernis
        {
            obstacle_handling_plan[current_obstacle_step] = OBS_LEFT;
            obstacle_handling_plan[current_obstacle_step + 1] = OBS_LEFT;
        }
        else // wir können weiterfahren
        {
            obstacle_handling_plan[current_obstacle_step] = OBS_FORWARD;
            obstacle_handling_plan[current_obstacle_step + 1] = OBS_LEFT;
        }
    }
    else if (current_obstacle_step == 2 || current_obstacle_step == 3)
    {
        if (obstacle_handling_plan[current_obstacle_step - 2] == OBS_LEFT) {
            obstacle_handling_plan[current_obstacle_step] = OBS_FORWARD;
            obstacle_handling_plan[current_obstacle_step + 1] = OBS_RIGHT;
            obstacle_handling_plan[current_obstacle_step + 2] = OBS_FORWARD;
            obstacle_handling_plan[current_obstacle_step + 3] = OBS_FORWARD;
            obstacle_handling_plan[current_obstacle_step + 4] = OBS_FORWARD;
            obstacle_handling_plan[current_obstacle_step + 5] = OBS_RIGHT;
            obstacle_handling_plan[current_obstacle_step + 6] = OBS_FORWARD;
            obstacle_handling_plan[current_obstacle_step + 7] = OBS_LEFT;
        }
        else if (Ultrasonic.distance_value < SAFE_DISTANCE) // es gibt auch hier ein hindernis
        {   
            // sehr viele annahmen über den Form des Obstacles wurden hier getroffen
            
            if (obstacle_handling_plan[current_obstacle_step - 2] == OBS_FORWARD) {
                obstacle_handling_plan[current_obstacle_step] = OBS_LEFT; 
                obstacle_handling_plan[current_obstacle_step + 1] = OBS_FORWARD;
                obstacle_handling_plan[current_obstacle_step + 2] = OBS_FORWARD;
                obstacle_handling_plan[current_obstacle_step + 3] = OBS_RIGHT;
                obstacle_handling_plan[current_obstacle_step + 4] = OBS_FORWARD;
                obstacle_handling_plan[current_obstacle_step + 5] = OBS_FORWARD;
                obstacle_handling_plan[current_obstacle_step + 6] = OBS_FORWARD;
                obstacle_handling_plan[current_obstacle_step + 7] = OBS_RIGHT;
                obstacle_handling_plan[current_obstacle_step + 8] = OBS_FORWARD;
                obstacle_handling_plan[current_obstacle_step + 9] = OBS_LEFT;
            }
            
        }
        else // wir können weiterfahren
        {
            obstacle_handling_plan[current_obstacle_step] = OBS_FORWARD;
            obstacle_handling_plan[current_obstacle_step + 1] = OBS_FORWARD;
            obstacle_handling_plan[current_obstacle_step + 2] = OBS_FORWARD;
            obstacle_handling_plan[current_obstacle_step + 3] = OBS_LEFT;
            obstacle_handling_plan[current_obstacle_step + 4] = OBS_FORWARD;
            obstacle_handling_plan[current_obstacle_step + 5] = OBS_RIGHT;
        }
    }
    else
    {
        return;
    }
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
        Balanced.Forward(FORWARD_SPEED);  // Langsam vorwärts
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
    unsigned int i = 0;
    while (i < 16) { // size of the obstacle handling plan
        obstacle_handling_plan[i] = OBS_NONE; // lösche alles was aktuell da drin ist
        //Serial.println("hello");
        i++;
    }
    obstacle_handling_plan[0] = OBS_RIGHT; //unser erster plan ist immer nach rechts zu drehen
    current_obstacle_step = 0;
}
void Function::Enter_Search_Mode()
{
    rgb.redOn();
    current_state = STATE_SEARCH;
}

// hier die lambda funktionen womit wir unsere plan liste befüllen
void Function::right() 
{
    Balanced.Right(RIGHT_SPEED);  // Nach rechts drehen
    
    if (millis() - turn_start_time >= TURN_180_TIME/2) { // nun 90 grad drehung statt 180 grad
        // Drehung abgeschlossen
        Balanced.Stop();
        obstacle_plan_step_finished = true;
    }
};
void Function::left() 
{
    Balanced.Left(LEFT_SPEED);  // Nach links drehen
    
    if (millis() - turn_start_time >= TURN_180_TIME/2) { // nun 90 grad drehung statt 180 grad
        // Drehung abgeschlossen
        Balanced.Stop();
        obstacle_plan_step_finished = true;
    }
};
void Function::straight() 
{
    Balanced.Forward(FORWARD_SPEED);  // Langsam nach vorne
    
    if (millis() - turn_start_time >= FORWARD_5CM_TIME) { 
        // Drehung abgeschlossen
        Balanced.Stop();
        obstacle_plan_step_finished = true;
    }
};

void Function::Execute_Obstacle_Step(ObstacleState plan_step) // this only exists because I couldn't get the libraries to work ;-;
{
    switch(plan_step) {
            
        case OBS_FORWARD:
            straight();
            break;
        
        case OBS_LEFT:
            left();
            break;
            
        case OBS_RIGHT:
            right();
            break;
        
        case OBS_NONE:
            break;
    
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

void IRLine::Send()
{
  static unsigned long ir_send_time;

  if (millis() - ir_send_time > 15)
  {
    for (int i = 0; i < 39; i++)
    { 
      digitalWrite(IR_SEND_PIN, LOW);
      delayMicroseconds(9);
      digitalWrite(IR_SEND_PIN, HIGH);
      delayMicroseconds(9);
    }
    ir_send_time = millis();
  }
}