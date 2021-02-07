#include <Wire.h>

const int FLEX_PIN = 18;
const float VCC = 4.9; // measured volatage of esp32 5v line
const float R_DIV = 47500.0; // measured resistance of 3.3k resistor

const float STRAIGHT_RESISTANCE = 37300.0; //resistance when straight
const float BEND_RESISTANCE = 90000.0; //resistance at 90 deg

void setup() {
  Serial.begin(9600);
  pinMode(FLEX_PIN, INPUT);

}

void loop() {
  //Read the ADC, and calculate voltage and resistance from it
  int flexADC = analogRead(FLEX_PIN);
  float flexV = flexADC*  VCC/1023.0;
  float flexR = R_DIV * (VCC/flexV - 1.0);
  Serial.println("Resistance: "+String(flexR)+" ohms");
  //use the calculated resistance to estimate the sensor's bend angle
  float angle = map(flexR, STRAIGHT_RESISTANCE, BEND_RESISTANCE, 0, 90.0);
  Serial.println("Bend: "+String(angle)+" degrees");
  Serial.println();

  delay(500);
}
