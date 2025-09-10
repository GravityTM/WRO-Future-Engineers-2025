#include <Arduino.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include "SparkFun_BNO08x_Arduino_Library.h"

BNO08x myIMU;
#define BNO08X_ADDR 0x4B

const int konumSayisi = 4;
int values[konumSayisi] = {0,0,0,0};
bool haveValues = false;


const int IN3 = 12;
const int IN4 = 14;
const int ENA = 27;  
const int PWM_CHANNEL = 0;

const int motorSpeed = 60;   
const int pwmFreq = 1000;     
const int pwmResolution = 8;  


Servo myServo;
const int servoPin = 13;
const int SERVO_CENTER = 95;  
int servoTarget = SERVO_CENTER;
int lastServo = SERVO_CENTER;

float Kp = 0.09;   
float Ki = 0.00005;
float Kd = 0.015;
float pid_integral = 0;
float pid_lastError = 0;
unsigned long pid_lastT = 0;
const int MAX_STEER_DELTA = 40; 
const int STEER_DEADBAND_MM = 30; 


const int FRONT_OBSTACLE_THRESHOLD = 500; 
const int WALL_MAX_DIST = 4000; 


const int SERVO_STEP_MS = 3; 
unsigned long lastServoMoveMillis = 0;


void setup() {
  Serial.begin(115200);
  Serial.println("Serial LiDAR ESP32 Robot Starting...");


  Wire.begin(21,22);
  if (!myIMU.begin(BNO08X_ADDR, Wire)) Serial.println("BNO08x not detected");
  else myIMU.enableRotationVector();


  ESP32PWM::allocateTimer(1);
  myServo.setPeriodHertz(50);
  myServo.attach(servoPin, 500, 2400);
  myServo.write(SERVO_CENTER);

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  ledcAttachPin(ENA, PWM_CHANNEL);
  ledcSetup(PWM_CHANNEL,pwmFreq,pwmResolution);

  pid_lastT = millis();
}


void handleLine(const String &s){
  String line = s; line.trim();
  if(line.length()==0) return;
  const int sabitKonumlar[]={90,180,270,360};
  bool konumBulundu[konumSayisi]={false,false,false,false};
  int tempValues[konumSayisi]={0,0,0,0};
  int pos=0;
  while(pos<line.length()){
    int colon=line.indexOf(':',pos);
    if(colon==-1) break;
    int comma=line.indexOf(',',colon);
    if(comma==-1) comma=line.length();
    String kStr=line.substring(pos,colon);
    String vStr=line.substring(colon+1,comma);
    int k=kStr.toInt(), v=vStr.toInt();
    for(int i=0;i<konumSayisi;i++) if(k==sabitKonumlar[i]) {tempValues[i]=v; konumBulundu[i]=true; break;}
    pos=comma+1;
  }
  // Assign values safely
  for(int i=0;i<konumSayisi;i++){ 
    int v=tempValues[i]; 
    if(v<0)v=0;
    if(v>WALL_MAX_DIST)v=WALL_MAX_DIST; 
    values[i]=v;
  }
  haveValues=true;

  // Debug print
  Serial.print("Parsed LiDAR: R90=");
  Serial.print(values[0]);
  Serial.print(" B180=");
  Serial.print(values[1]);
  Serial.print(" L270=");
  Serial.print(values[2]);
  Serial.print(" F360=");
  Serial.println(values[3]);
}


void computeSteeringFromWalls(){
  if(!haveValues) return;
  int right=values[0], left=values[2], front=values[3];

  
  if(front>0 && front<FRONT_OBSTACLE_THRESHOLD){
    if(left>right+50) servoTarget=SERVO_CENTER-30;
    else if(right>left+50) servoTarget=SERVO_CENTER+30;
    else servoTarget=SERVO_CENTER;
    Serial.println("Front obstacle detected!");
    return;
  }

  float error=(float)left-(float)right;
  if(fabs(error)<STEER_DEADBAND_MM){ servoTarget=SERVO_CENTER; pid_integral=0; pid_lastError=0; return;}
  
  unsigned long now=millis();
  float dt=(now-pid_lastT)/1000.0; if(dt<=0) dt=0.001;
  pid_integral+=error*dt;
  float derivative=(error-pid_lastError)/dt;
  float output_deg=Kp*error+Ki*pid_integral+Kd*derivative;
  pid_lastError=error; pid_lastT=now;
  
  int rawTarget=SERVO_CENTER+(int)round(output_deg);
  if(rawTarget<SERVO_CENTER-MAX_STEER_DELTA) rawTarget=SERVO_CENTER-MAX_STEER_DELTA;
  if(rawTarget>SERVO_CENTER+MAX_STEER_DELTA) rawTarget=SERVO_CENTER+MAX_STEER_DELTA;
  servoTarget=rawTarget;

  Serial.print("PID Steering -> target: ");
  Serial.println(servoTarget);
}


void updateServoSmooth(){
  unsigned long now=millis();
  if(now-lastServoMoveMillis<SERVO_STEP_MS) return;
  lastServoMoveMillis=now;
  if(lastServo<servoTarget) lastServo++;
  else if(lastServo>servoTarget) lastServo--;
  myServo.write(lastServo);
}


void runMotorsF(int speed){ digitalWrite(IN3,HIGH); digitalWrite(IN4,LOW); ledcWrite(PWM_CHANNEL,speed);}
void stopMotors(){ ledcWrite(PWM_CHANNEL,0); digitalWrite(IN3,LOW); digitalWrite(IN4,LOW);}

void loop(){
  
  if(Serial.available()){
    String line=Serial.readStringUntil('\n');
    handleLine(line);
    computeSteeringFromWalls();
    updateServoSmooth();
    runMotorsF(motorSpeed);
  } else {
    stopMotors();
  }
}
