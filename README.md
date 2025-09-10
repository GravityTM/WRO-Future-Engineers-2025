Table of Contents
--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
## Table of Contents

- [Team](#The-Team)
- [Video of the Robot](#Video-of-the-Robot)
- [Photos of Robot](#Photos-of-Robot)
- [3D Model of Robot](#3D-Model-of-Robot)
- [Electronic Circuit](#Electronic-Circuit)
- [ESP32 LiDAR Control Code](#ESP32-LiDAR-Control-Code)
- [YOLO Cube Detection & Distance Estimation](#YOLO-Cube-Detection-and-Distance-Estimation)
- [ESP32 + Servo Control for Jetson LiDAR Data](#esp32-servo-control-for-jetson-lidar-data)




# The Team
-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
### Banu Isgandarli

Hello! My name is Banu, and I am 15 years old. To date, I have participated in four competitions, and this year marks my first foray into the WRO Future Engineers category. In my free time, I am passionate about robotics, programming, and playing the violin. I also enjoy exploring new concepts and continuously developing my skills. Additionally, I have a strong interest in chess and Formula 1, which reflect my enthusiasm for both strategic thinking and dynamic problem-solving.🎀🏎




### Jannat Ozdemir


<img src="https://github.com/user-attachments/assets/7d52f428-0e74-4901-9b23-cc38ac3ba7bd" width="200px" height="200px"/>

My name is Jannat Özdemir, I am 16 years old.My zodiac sign is Sagittarius. In my free time, I enjoy horse riding, reading books, and learning new languages. I also love traveling around different countries. I have previously participated in the LEGO Line category. I am passionate about cybersecurity.l can make the best desserts in the world My signature phrase is: “We will beat everyone"💅💎







### Fatima Alizade

<img src="https://github.com/user-attachments/assets/9bb84605-dea2-4183-8ffc-12d16d6cfe33" width="100px" height="200px"/>

Hello, my name is Fatima Elizade. I am 16 years old. I study in high school. I like programming. I enjoy playing games and also creating my own games. I know 3 programming languages.I love eating food.I like coconut🥥🫶🏿






# Video of the Robot
Open Round-->https://www.youtube.com/watch?v=aMErQvMpUjY

# Photos of Robot
| Top  | Side |
|--------------|---------------|
| <img src="https://github.com/user-attachments/assets/7f6000c9-395f-4648-aea6-9cd50357c543" width="500px"/> | <img src="https://github.com/user-attachments/assets/f95d4301-5aab-4dcf-ae27-32852ed6ee1c"  width="500px"/> |
| Front  | Back |
|              |               |
| <img src="https://github.com/user-attachments/assets/17ec1577-6b1a-4b27-a598-c34dd2752b3c" width="500px"/> | <img src="https://github.com/user-attachments/assets/50a999c8-c31d-4f95-b573-afb8f3041006" width="500"/> |

-----------------------------------------------------------------------



# 3D Model of Robot
-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
| Top 3D Model | Side 3D Model | Front 3D Model | Isometric 3D Model |
|--------------|---------------|----------------|----------------|
| <img src="https://github.com/user-attachments/assets/3022cd89-6f7f-4c03-b50d-51091934c3ac" width="200"/> | <img src="https://github.com/user-attachments/assets/42a4a9e2-2a62-4ac1-b08d-6e0914a790e9" width="200px" height="200px"/> | <img src="https://github.com/user-attachments/assets/97e87bd3-68e6-482d-ac02-700d0cf6718d" width="200"/> | <img src="https://github.com/user-attachments/assets/02416aa0-53ec-409b-bdfa-e27084d24a1f" width="200"/> |
-----------------------------------------------------------------------

# Electronic Circuit
--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
<img src="https://github.com/user-attachments/assets/5a9c56da-8c2a-45b3-afc5-0a92218413eb" width="700px" height="700px"/>




## 🔧 Components

* *NVIDIA Jetson Nano* – main AI and vision processing unit.
* *ESP32* – microcontroller for sensor reading and motor control.
* *D500 Lidar (A1/A2)* – for mapping and distance measurement.
* *CSI Camera* – object detection and vision input.
* *L298N Motor Driver* – controls DC motors.
* *DC Gear Motor (yellow)* – provides robot movement.
* *Servo Motor (SG90/MG90S)* – steering or mechanical actuation.
* *Step-Down Buck Converters (LM2596)* – voltage regulation for modules.
* *Battery Pack (Li-Po or Li-Ion)* – main power source.

---

## ⚡ Power Distribution

* All modules share a *common GND*.
* Battery → Step-Down → 5V output:

  * Jetson Nano
  * ESP32
  * Servo motor
* Battery → L298N Motor Driver → DC motor

---
# ESP32 LiDAR Control Code

### ESP32

* *Servo PWM* → GPIO PWM output.
* *L298N IN1, IN2, IN3, IN4* → GPIO pins for motor control.
* *UART (TX/RX)* → Serial communication with Jetson Nano.

### Jetson Nano

* *USB Camera* → USB port.
* *D500 Lidar* → USB/UART port.
* *ESP32 Communication* → UART (TX/RX).

---

## 📲 System Workflow

1. *D500 Lidar* scans the environment for mapping.
2. *Camera* provides video input for AI vision.
3. *ESP32* collects sensor data and controls motors/servos.
4. *Jetson Nano* processes AI models and sends decisions to ESP32.
5. *Motor Driver (L298N)* drives the DC motor based on ESP32 signals.

---

## 🚀 Operating Principle

* When powered on, ESP32 and Jetson Nano establish serial communication.
* Jetson Nano runs AI tasks (object detection, path planning).
* ESP32 executes low-level controls (distance checks, motor PWM, servo).
* Together, they achieve autonomous robot navigation for WRO challenges.

---





# YOLO Cube Detection and Distance Estimation

This project uses a **YOLOv8 model** and OpenCV to detect cubes (green and red), estimate their distance from the camera, and show navigation hints (`⬅ LEFT` or `RIGHT ➡`) depending on the closest cube.  
---

## 🚀 Features
- Real-time detection of red and green cubes  
- Distance estimation using the pinhole camera model  
- Vertical guide lines for alignment  
- Navigation hints based on the closest cube  

---

## 📦 Requirements
Install the dependencies:

```bash
pip install ultralytics opencv-python
```

---

## ⚙️ Camera Calibration
The script uses calibration values to calculate distances:

```python
FOCAL_LENGTH = 846.0   # px (from calibration)
REAL_HEIGHT  = 10.0    # cm (real cube height)
LINE_DIST_CM = 20.0    # cm (distance between guiding lines)
```

You can change these values for your own camera setup.

---

## ▶️ Usage

Run the script:

```bash
python detect_cubes.py
```

Press **Q** to quit.

---

## 🔍 Code

```python
from ultralytics import YOLO
import cv2

# Model
model = YOLO('/home/farad/Desktop/red-green/my_model/my_model.pt')

# Camera calibration
FOCAL_LENGTH = 846.0    # px
REAL_HEIGHT  = 10.0     # cm (cube height)
LINE_DIST_CM = 20.0     # distance between green lines (cm)

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    h_img, w_img = frame.shape[:2]
    x_center = w_img // 2  # camera center

    # Base lines (reference at Z=50 cm)
    Z_ref = 50.0  
    half_span_px = (LINE_DIST_CM/2 * FOCAL_LENGTH) / Z_ref

    left_line_x  = int(x_center - half_span_px)
    right_line_x = int(x_center + half_span_px)

    # YOLO detection
    results = model(frame, conf=0.5)
    annotated_frame = results[0].plot()

    # ---------- Find the closest cube ----------
    closest_obj = None
    min_dist = float("inf")
    direction_text = ""

    for box in results[0].boxes:
        x1, y1, x2, y2 = box.xyxy[0].int().tolist()
        width  = x2 - x1
        height = y2 - y1
        cx     = x1 + width // 2
        cy     = y1 + height // 2

        if height <= 0:
            continue

        # -------- Distance estimation --------
        Z_cm = (FOCAL_LENGTH * REAL_HEIGHT) / float(height)

        # ---------- Object class ----------
        cls_id = int(box.cls[0].item())
        class_name = results[0].names[cls_id]

        # --- Update if this is the closest cube ---
        if Z_cm < min_dist:
            min_dist = Z_cm
            closest_obj = class_name

        # --- Visualization for each cube ---
        cv2.putText(annotated_frame, f"{class_name}", (x1, max(0, y1-30)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)
        cv2.putText(annotated_frame, f"D:{Z_cm:.1f}cm", (x1, max(0, y1-10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)
        cv2.drawMarker(annotated_frame, (cx, cy), (0,0,255),
                       cv2.MARKER_CROSS, 20, 2)
        cv2.putText(annotated_frame, f"({cx},{cy})", (cx+10, cy),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200,200,0), 2)

    # --- Final navigation hint based on closest cube ---
    if closest_obj == "green_box":
        direction_text = "⬅ LEFT"
    elif closest_obj == "red_box":
        direction_text = "RIGHT ➡"

    # Vertical lines
    cv2.line(annotated_frame, (left_line_x, 0), (left_line_x, h_img), (0,255,0), 2)
    cv2.line(annotated_frame, (right_line_x, 0), (right_line_x, h_img), (0,255,0), 2)
    cv2.line(annotated_frame, (x_center, 0), (x_center, h_img), (0,0,255), 2)

    # Navigation hint
    if direction_text != "":
        cv2.putText(annotated_frame, direction_text, (50, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 200, 255), 3)

    cv2.imshow("YOLO Cubes + Distances", annotated_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
```

---

# ESP32 + Servo Control for Jetson LiDAR Data
```cpp
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
```
