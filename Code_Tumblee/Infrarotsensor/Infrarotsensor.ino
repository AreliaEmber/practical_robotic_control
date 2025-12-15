// TCRT5000 Sensor Test - Einfache Version
// Zeigt nur die Sensorwerte an

const int SENSOR_LEFT = A0;
const int SENSOR_RIGHT = A1;

// Schwellwert zum Testen (anpassen!)
const int THRESHOLD = 512;

void setup() {
  Serial.begin(9600);
  
  pinMode(SENSOR_LEFT, INPUT);
  pinMode(SENSOR_RIGHT, INPUT);
  delay(2000);
}

void loop() {
  // Sensoren auslesen (0-1023)
  int leftValue = analogRead(SENSOR_LEFT);
  int rightValue = analogRead(SENSOR_RIGHT);
  
  // Interpretation: Über oder unter Schwellwert?
  bool leftOnBlack = (leftValue > THRESHOLD);
  bool rightOnBlack = (rightValue > THRESHOLD);
  
  // === Ausgabe 1: Rohe Werte ===
  Serial.print("Links: ");
  Serial.print(leftValue);
  Serial.print("\t");
  
  Serial.print("Rechts: ");
  Serial.print(rightValue);
  Serial.print("\t");
  
  // === Ausgabe 2: Balken-Anzeige ===
  Serial.print("| L:[");
  printBar(leftValue, 20);
  Serial.print("] R:[");
  printBar(rightValue, 20);
  Serial.print("] |");
  
  // === Ausgabe 3: Status ===
  Serial.print("\t");
  Serial.print(leftOnBlack ? "SCHWARZ" : "WEISS  ");
  Serial.print(" | ");
  Serial.print(rightOnBlack ? "SCHWARZ" : "WEISS  ");
  
  Serial.println();
  
  delay(200);  // 5x pro Sekunde aktualisieren
}

// Hilfsfunktion: Zeigt Wert als Balken (0-1023 -> 0-20 Zeichen)
void printBar(int value, int maxLength) {
  int bars = map(value, 0, 1023, 0, maxLength);
  
  for (int i = 0; i < bars; i++) {
    Serial.print("=");
  }
  for (int i = bars; i < maxLength; i++) {
    Serial.print(" ");
  }
}