const int LedPin1=4;

const int LedPin2=2;

const int LedPin3=0;
const int LedPin4=5;

void setup() {
  // put your setup code here, to run once:

    pinMode(LedPin1,OUTPUT);
    pinMode(LedPin2,OUTPUT);
    pinMode(LedPin3,OUTPUT);
    pinMode(LedPin4,OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
    digitalWrite(LedPin1,HIGH);
    digitalWrite(LedPin2,HIGH);
    digitalWrite(LedPin3,HIGH);
    digitalWrite(LedPin4,HIGH);


    delay(900);
    digitalWrite(LedPin1,LOW);
    digitalWrite(LedPin2,LOW);
    digitalWrite(LedPin3,LOW);
    digitalWrite(LedPin4,LOW);


    delay(500);
}
