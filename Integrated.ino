#include<elapsedMillis.h>
#include <dht.h>
// 2, A0, 10, 11, 9, 12, 8, 3, 5, 6
// 0, 1, 4, 7
//Flow-meter
byte statusLed = 13; // -------------------------------------Pin------------------------------------------
byte sensorInterrupt = 0;  // 0 = digital pin 2
byte sensorPin = 2; // DO NOT CHANGE FROM PIN 2  -------------------------------------Pin------------------------------------------
float calibrationFactor = 4.5;
volatile byte pulseCount;  
float flowRate;
unsigned int flowMilliLitres;
unsigned long totalMilliLitres;
unsigned long oldTime;


//Ambient Light
const int analogInPin = A0;  // Analog input pin that the potentiometer is attached to  -------------------------------------Pin------------------------------------------
const int analogOutPin = 6; // Analog output pin that the LED is attached to -------------------------------------Pin------------------------------------------
int sensorValue = 0;        // value read from the pot
int outputValue = 0;        // value output to the PWM (analog out)


//Geyser
int relay = 12; //-------------------------------------Pin------------------------------------------


//Exhaust
dht DHT;
#define DHT11_PIN 9 //-------------------------------------Pin------------------------------------------
int oPin = 8; // -------------------------------------Pin------------------------------------------
int chk, val = 0;
unsigned long oldTimeExhaust;

//Presence
const int trigPin = 6; //-------------------------------------Pin------------------------------------------
const int echoPin = 5; //--------- ----------------------------Pin------------------------------------------
const int pirinput = 3; // -------------------------------------Pin------------------------------------------
//const int breakEm = 6; // Virtual Pin 
elapsedMillis Emergency;
long duration;
double distance;
int presence = 0;

void setup() {
  //Serial monitor initialisation
  Serial.begin(9600);
  
  
  //Flow-meter
  pinMode(statusLed, OUTPUT);
  digitalWrite(statusLed, HIGH);  // We have an active-low LED attached
  pinMode(sensorPin, INPUT);
  digitalWrite(sensorPin, HIGH);
  pulseCount        = 0;
  flowRate          = 0.0;
  flowMilliLitres   = 0;
  totalMilliLitres  = 0;
  oldTime           = 0;
  // The Hall-effect sensor is connected to pin 2 which uses interrupt 0.
  // Configured to trigger on a FALLING state change (transition from HIGH
  // state to LOW state)
  attachInterrupt(sensorInterrupt, pulseCounter, FALLING);

  
  //Geyser
  pinMode(relay,OUTPUT);
  
  
  //Presence
  //pinMode(breakEm, INPUT);
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  pinMode(pirinput, INPUT);
  presence = 0;
}
// Setup() End

//Functions

//Flow-meter
void pulseCounter(){
  // Increment the pulse counter
  pulseCount++;
}

int appInAmbient = 0;
int appInGeyser = 0;
int appInExhaust = 0;
int breakEm = 0;
int toiletSeat = 0;
void loop() {
  
  //Washroom Empty
  if( presence == 0 ){  
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH);
    distance= duration*0.034/2;
    if(distance<60){
      int previous_millis = millis();
      int current_millis = millis();
      while( current_millis - previous_millis < 10000){
        if(digitalRead(pirinput) == HIGH){
          presence = 1;
          Emergency = 0;
          break;
        }
        current_millis = millis();
      }
    }
    
    //Turn Off Light
    analogWrite(analogOutPin, 0);
    //Turn Off Exhaust
    digitalWrite(oPin, HIGH);
    //Geyser
    if(appInGeyser == 1){
      digitalWrite(relay,LOW);
    }
    else{
      digitalWrite(relay,HIGH);
    }
    //Flow
    if((millis() - oldTime) > 1000){    // Only process counters once per second 
      detachInterrupt(sensorInterrupt);
      flowRate = ((1000.0 / (millis() - oldTime)) * pulseCount) / calibrationFactor;
      oldTime = millis();
      flowMilliLitres = (flowRate / 60) * 1000;
      totalMilliLitres += flowMilliLitres;
      unsigned int frac;        
      frac = (flowRate - int(flowRate)) * 10;
      // Reset the pulse counter so we can start incrementing again
      pulseCount = 0;
      // Enable the interrupt again now that we've finished sending output
      attachInterrupt(sensorInterrupt, pulseCounter, FALLING);
    }
    if(flowRate > 0){
      //Leak
    }
  }
  
  //Washroom Occupied
  else{
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH);
    distance= duration*0.034/2;
    if(digitalRead(pirinput) == HIGH)
      Emergency = 0; 
    while(Emergency > 20000)
    {
      if(digitalRead(breakEm) == HIGH)
      {
        Emergency = 0;
        break;
      }
      //Emergency detected here - Send notif
    }
    if(distance<60)
    {
      int previous_millis = millis();
      int current_millis = millis();
      int flag = 0;
      while( current_millis - previous_millis < 10000)
      {
        if(digitalRead(pirinput) == HIGH)
        {
          flag = 1;
          Emergency = 0;
          break;
        }
        current_millis = millis();
      }
      if(flag == 0)
      {
      presence = 0;
       
      }
    }
    //Flow
    if((millis() - oldTime) > 1000){    // Only process counters once per second 
      detachInterrupt(sensorInterrupt);
      flowRate = ((1000.0 / (millis() - oldTime)) * pulseCount) / calibrationFactor;
      oldTime = millis();
      flowMilliLitres = (flowRate / 60) * 1000;
      totalMilliLitres += flowMilliLitres;
      unsigned int frac;        
      frac = (flowRate - int(flowRate)) * 10;
      // Reset the pulse counter so we can start incrementing again
      pulseCount = 0;
      // Enable the interrupt again now that we've finished sending output
      attachInterrupt(sensorInterrupt, pulseCounter, FALLING);
    }

    //Ambient-Light
    if(appInAmbient == -1){
      // read the analog in value:
      sensorValue = analogRead(analogInPin);
      // map it to the range of the analog out:
      outputValue = map(sensorValue, 0, 1023, 0, 255);
      // change the analog out value:
      analogWrite(analogOutPin, outputValue);
    }
    else{
      // read the analog in value:
      //sensorValue = analogRead(analogInPin);
      // map it to the range of the analog out:
      outputValue = map(appInAmbient, 0, 1023, 0, 255);
      // change the analog out value:
      analogWrite(analogOutPin, outputValue);
    } 

    //Geyser
    if(appInGeyser == 1){
      digitalWrite(relay,LOW);
    }
    else{
      digitalWrite(relay,HIGH);
    }

    //Exhaust
    
    if(appInExhaust == -1){
      if(val > 80){
        digitalWrite(oPin,LOW);
        if((millis() - oldTimeExhaust) > 10000){
          digitalWrite(oPin,HIGH);
          val = 0;
          if((millis() - oldTimeExhaust) > 20000){
            chk = DHT.read11(DHT11_PIN);
            val = DHT.humidity;
            oldTimeExhaust = millis();
          }
        }
      }
      else if(val > 75){
        digitalWrite(oPin,LOW);
        if((millis() - oldTimeExhaust) > 8000){
          digitalWrite(oPin,HIGH);
          val = 0;
          if((millis() - oldTimeExhaust) > 20000){
            chk = DHT.read11(DHT11_PIN);
            val = DHT.humidity;
            oldTimeExhaust = millis();
          }
        }
      }
      else if(val > 70)
      {
        digitalWrite(oPin,LOW);
        if((millis() - oldTimeExhaust) > 5000){
          digitalWrite(oPin,HIGH);
          val = 0;
          if((millis() - oldTimeExhaust) > 20000){
            chk = DHT.read11(DHT11_PIN);
            val = DHT.humidity;
            oldTimeExhaust = millis();
          }
        }
      }
      else{
        digitalWrite(oPin, HIGH);
        if((millis() - oldTimeExhaust) > 20000){
            chk = DHT.read11(DHT11_PIN);
            val = DHT.humidity;
            oldTimeExhaust = millis();
          }
      }
    }
    else if(appInExhaust == 1){
      digitalWrite(oPin, LOW);
    }
    else{
      digitalWrite(oPin, HIGH);
    }
  
  }
}
