#pragma once

#ifndef Pins_h
#define Pins_h

// ANALOGE PINS

#define LEFT_RECEIVE_PIN A0 // A0 – linker IR-Empfänger (Obstacle) --> TCRT5000
#define RIGHT_RECEIVE_PIN A1 // A1 – rechter IR-Empfänger (Obstacle) --> TCRT5000
#define VOL_MEASURE_PIN A2 // A2 – Batteriespannungsmessung
#define ECHO_PIN A3 // A3 – Ultraschall ECHO


// DIGITALE SENSORPINS

#define TRIG_PIN 11 // D11 – Ultraschall TRIG → steuert den Ultraschallsensor (10 µs HIGH-Puls)
#define IR_SEND_PIN 9 // D9 – IR Senden → treibt die IR-Sende-LED (38 kHz Burst) → wird für Obstacle-/Follow-IR verwendet

#define RECV_PIN 9 // D9 (Alias) → identisch zu IR_SEND_PIN


// MOTORTREIBER

#define PWMA_LEFT 5 // D5 – PWM linker Motor → Geschwindigkeitsregelung linker Motor
#define PWMB_RIGHT 6 // // D6 – PWM rechter Motor → Geschwindigkeitsregelung rechter Motor

#define AIN1 7 // D7 – AIN1 (Motor-Richtung) → Drehrichtung Motor
#define BIN1 12 // D12 – BIN1 (Motor-Richtung) → Zweiter Richtungs-Pin des Motortreibers

#define STBY_PIN 8 // D8 – STBY (Motor Enable) → aktiviert/deaktiviert den Motortreiber


// ENCODER (Motor)

#define ENCODER_LEFT_A_PIN 2 // D2 – linker Encoder Kanal A
#define ENCODER_RIGHT_A_PIN 4 // D4 – rechter Encoder Kanal A

// RGB / UI

#define RGB_PIN 3 // D3 – RGB LED Datenleitung → steuert WS2812 / NeoPixel LED(s)
#define NUMPIXELS 4 // Anzahl RGB LEDs
#define KEY_MODE 10 // D10 – Mode-Taste / Modus-Umschaltung

#endif
