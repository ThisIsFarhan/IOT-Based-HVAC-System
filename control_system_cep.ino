#include "DHT.h"
#include <WiFi.h>
#define DHTTYPE DHT22
#define DHTPIN 27

#define wifi_ssid ""
#define wifi_pass ""

DHT dht(DHTPIN, DHTTYPE);

// Motor A
int motor1Pin1 = 32; 
int motor1Pin2 = 33; 
int enable1Pin = 26;

//Bulb
int bulbPin3 = 12;
int bulbPin4 = 14;
int enable2Pin = 25;

// Setting PWM properties
const int freq = 80000;
const int pwmChannel = 0;
const int resolution = 8;
// int dutyCycle = 200;

//bulb
const int freq1 = 5000;
const int pwmChannel1 = 1;
const int resolution1 = 8;

int fanFlag = 0;
int bulbFlag = 0;


double dt, last_time;
double integral, previous, output = 0;
double kp, ki, kd;
double setpoint = 33;




void setup() {

  Serial.begin(115200);
  WiFi.begin(wifi_ssid, wifi_pass);
  Serial.print("Connecting to Wi-Fi");
  while(WiFi.status() != WL_CONNECTED){
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.print(WiFi.localIP());
  Serial.println();
 
  // sets the pins as outputs:
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(enable1Pin, OUTPUT);

  pinMode(bulbPin3, OUTPUT);
  pinMode(bulbPin4, OUTPUT);
  pinMode(enable2Pin, OUTPUT);


  
  // configure LED PWM functionalitites
  ledcSetup(pwmChannel, freq, resolution);
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(enable1Pin, pwmChannel);

  Serial.begin(115200);

  // testing
  Serial.print("Testing DC Motor...");

  dht.begin();

  ledcSetup(pwmChannel1, freq, resolution);
  ledcAttachPin(enable2Pin, pwmChannel1);


  // const int INPUT_PIN = A0;
  // const int OUTPUT_PIN = DD3;
  

  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);

  digitalWrite(bulbPin3, HIGH);
  digitalWrite(bulbPin4, LOW);

  kp = 0.8;
  ki = 0.20;
  kd = 0.001;
  last_time = 0;
}

void loop() {
  double now = millis();
  dt = (now - last_time)/1000.00;
  last_time = now;

  // double actual = map(analogRead(INPUT_PIN), 0, 1024, 0, 255);
  double actual = map(dht.readTemperature(),25,40,0,255);
  double error = map(setpoint,25,40,0,255) - actual;


  if(error > 0){
    bulbFlag = 1;
    fanFlag = 0;
  }
  else{
    bulbFlag = 0;
    fanFlag = 1;
  }

  output = pid(error);

  if(fanFlag == 1 && bulbFlag == 0){
    ledcWrite(pwmChannel, output);
    ledcWrite(pwmChannel1, 0);
  }
  else if(fanFlag == 0 && bulbFlag == 1){
    ledcWrite(pwmChannel1, output);
    ledcWrite(pwmChannel, 0);
  }


  Serial.print(actual);
  Serial.print(",");
  Serial.print(map(setpoint,25,40,0,255));
  Serial.println();


  delay(300);
}


double pid(double error)
{
  double proportional = error;
  integral += error * dt;
  double derivative = (error - previous) / dt;
  previous = error;
  double output = (kp * proportional) + (ki * integral) + (kd * derivative);
  output = abs(output);
  if(output > 255){
    output = 255;
  }
  return output;
}