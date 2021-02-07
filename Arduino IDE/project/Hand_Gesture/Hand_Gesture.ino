#include <Wire.h>
#include <Adafruit_Sensor.h>    // Adafruit  sensor library
#include <Adafruit_ADXL345_U.h>  // ADXL345 library

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified();   // ADXL345 Object

const int FLEX1_PIN = 18;
const int FLEX2_PIN = 19;
const float VCC = 4.9; // measured volatage of esp32 5v line
const float R_DIV = 47500.0; // measured resistance of 3.3k resistor

const float STRAIGHT_RESISTANCE = 37300.0; //resistance when straight
const float BEND_RESISTANCE = 90000.0; //resistance at 90 deg

void setup() {

  Serial.begin(9600);
  pinMode(FLEX1_PIN, INPUT);
  pinMode(FLEX2_PIN, INPUT);
  if(!accel.begin())   // if ASXL345 sensor not found
  {
    Serial.println("ADXL345 not detected");
    while(1);
  }

}

void loop() {

 sensors_event_t event;
 accel.getEvent(&event);
 Serial.print("X: ");
 Serial.print(event.acceleration.x);
 Serial.print("  ");
  Serial.print("Y: ");
 Serial.print(event.acceleration.y);
 Serial.print("  ");
  Serial.print("Z: ");
 Serial.print(event.acceleration.z);
 Serial.print("  ");
 Serial.println("m/s^2 ");
 delay(500);

 //Read the ADC, and calculate voltage and resistance from it
  int flexADC1 = analogRead(FLEX1_PIN);
  int flexADC2 = analogRead(FLEX2_PIN);
  float flex1V = flexADC1*  VCC/1023.0;
  float flex1R = R_DIV * (VCC/flex1V - 1.0);

  float flex2V = flexADC2*  VCC/1023.0;
  float flex2R = R_DIV * (VCC/flex2V - 1.0);
  Serial.println("Resistance1: "+String(flex1R)+" ohms");
  Serial.println("Resistance2: "+String(flex2R)+" ohms");

  
  //use the calculated resistance to estimate the sensor's bend angle
  float flex1_val = map(flex1R, STRAIGHT_RESISTANCE, BEND_RESISTANCE, 0, 90.0);
  float flex2_val = map(flex2R, STRAIGHT_RESISTANCE, BEND_RESISTANCE, 0, 90.0);

  if(flex1_val > 250 && event.acceleration.x > -0.55 && event.acceleration.x < -0.40) 
  {
    Serial.println("I need Water");
    delay(2000);
  }
  if(flex1_val > 150 && flex2_val <250 && event.acceleration.x > -0.55 && event.acceleration.x < -0.40) 
  {
    Serial.println("I need food");
    //bluetooth code to be added here
    delay(2000);
  }
  if(flex1_val > 50 && flex2_val <150 && event.acceleration.x > -0.55 && event.acceleration.x < -0.40) 
  {
    Serial.println("I need Medicine");
    //bluetooth code to be added here
    delay(2000);
  }
  if(flex1_val > 250 && flex2_val > 250 && event.acceleration.x > -0.55 && event.acceleration.x < -0.40) 
  {
    Serial.println("I need Help");
    //bluetooth code to be added here
    delay(2000);
  }
  
  Serial.println("Bend1: "+String(flex1_val)+" degrees");
  Serial.println("Bend2: "+String(flex2_val)+" degrees");

  Serial.println();
  delay(500);
}
