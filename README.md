Table of Contents
-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
- The Team


The Team
-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
### Banu Isgandarli

<img src="https://github.com/user-attachments/assets/d1633134-9a48-4e6a-9c2a-a42ba5db09b0" width="400px" height="500px"/>


Hello! My name is Banu, and I am 15 years old. To date, I have participated in four competitions, and this year marks my first foray into the WRO Future Engineers category. In my free time, I am passionate about robotics, programming, and playing the violin. I also enjoy exploring new concepts and continuously developing my skills. Additionally, I have a strong interest in chess and Formula 1, which reflect my enthusiasm for both strategic thinking and dynamic problem-solving.🎀🏎




### Jannat Ozdemir


<img src="https://github.com/user-attachments/assets/7d52f428-0e74-4901-9b23-cc38ac3ba7bd" width="300px" height="300px"/>
My name is Jannat Özdemir, I am 16 years old.My zodiac sign is Sagittarius. In my free time, I enjoy horse riding, reading books, and learning new languages. I also love traveling around different countries. I have previously participated in the LEGO Line category. I am passionate about cybersecurity.l can make the best desserts in the world My signature phrase is: “We will beat everyone"💅💎




### Fatima Alizade

<img src="https://github.com/user-attachments/assets/9bb84605-dea2-4183-8ffc-12d16d6cfe33" width="300px" height="500px"/>
Hello, my name is Fatima Elizade. I am 16 years old. I study in high school. I like programming. I enjoy playing games and also creating my own games. I know 3 programming languages.I love eating food.I like coconut🥥🫶🏿




3D Model of Robot
-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
| Top 3D Model | Side 3D Model | Front 3D Model | Isometric 3D Model |
|--------------|---------------|----------------|----------------|
| <img src="https://github.com/user-attachments/assets/3022cd89-6f7f-4c03-b50d-51091934c3ac" width="200"/> | <img src="https://github.com/user-attachments/assets/42a4a9e2-2a62-4ac1-b08d-6e0914a790e9" width="200"/> | <img src="https://github.com/user-attachments/assets/97e87bd3-68e6-482d-ac02-700d0cf6718d" width="200"/> | <img src="https://github.com/user-attachments/assets/02416aa0-53ec-409b-bdfa-e27084d24a1f" width="200"/> |
-----------------------------------------------------------------------
YOLO Cube Detection with Distance Estimation

This project uses a YOLOv8 model to detect cubes (green and red), estimate their distance from the camera, and display a navigation hint (LEFT or RIGHT) based on the closest cube.

Features

Real-time cube detection with YOLO

Camera calibration to calculate distances in centimeters

Draws detection boxes, center markers, and distance labels on each cube

Vertical guide lines for camera alignment

Shows navigation hint (⬅ LEFT or RIGHT ➡) depending on the closest cube’s color

Requirements

Install dependencies before running the script:

pip install ultralytics opencv-python

Camera Calibration Parameters

In the script, you can adjust the following values according to your camera setup:

FOCAL_LENGTH = 846.0   # px (calculated from camera calibration)
REAL_HEIGHT  = 10.0    # cm (real cube height)
LINE_DIST_CM = 20.0    # cm (distance between green guiding lines)

Usage

Run the script with:

python detect_cubes.py

Controls

Press Q to quit

Code Overview

YOLO model is loaded from a custom .pt file:

model = YOLO('/home/farad/Desktop/red-green/my_model/my_model.pt')


Each detected cube is analyzed:

Distance is calculated using the pinhole camera model:

𝑍
𝑐
𝑚
=
𝐹
𝑜
𝑐
𝑎
𝑙
 
𝐿
𝑒
𝑛
𝑔
𝑡
ℎ
×
𝑅
𝑒
𝑎
𝑙
 
𝐻
𝑒
𝑖
𝑔
ℎ
𝑡
𝐵
𝑜
𝑢
𝑛
𝑑
𝑖
𝑛
𝑔
 
𝐵
𝑜
𝑥
 
𝐻
𝑒
𝑖
𝑔
ℎ
𝑡
Z
cm
	​

=
Bounding Box Height
Focal Length×Real Height
	​


The cube with the minimum distance is selected as the reference.

If it is green_box → ⬅ LEFT is displayed.

If it is red_box → RIGHT ➡ is displayed.

Example Output

Bounding boxes with labels (class, distance)

Cube center marker with (x, y) coordinates

Vertical guide lines (green + red)

Direction text (LEFT or RIGHT)

Demo Screenshot

(You can add your own captured screenshot here)

![Demo](your_screenshot.png)


Do you want me to also make a short explanation of how to calculate FOCAL_LENGTH (so you can recalibrate if needed), or just keep the README like this?

You said:
wro ucun readme ele
ChatGPT said:

Super 👍 WRO (World Robot Olympiad) üçün bu kodu istifadə etməyinizə görə README faylını WRO-ya uyğun formada hazırladım. Burada həm yarış konteksti, həm də kodun məqsədi izah olunub. 👇

WRO Future Engineers – Cube Detection & Navigation

This project is developed for the WRO Future Engineers category.
It uses a YOLOv8 model and camera calibration to:

Detect red and green cubes on the field

Estimate their distance from the robot

Give a navigation hint (LEFT / RIGHT) based on the closest cube

This helps the robot make real-time decisions during the run.

🚀 Features

Real-time detection of green_box and red_box

Distance estimation using the pinhole camera model

Vertical guide lines for robot alignment

Navigation hint (⬅ LEFT or RIGHT ➡) according to WRO rules

Works with webcam or robot camera

📦 Requirements

Install the necessary packages before running:

pip install ultralytics opencv-python

⚙️ Camera Calibration

The script uses camera calibration values to measure distance correctly:

FOCAL_LENGTH = 846.0   # px (from camera calibration)
REAL_HEIGHT  = 10.0    # cm (real cube height on WRO field)
LINE_DIST_CM = 20.0    # cm (distance between guiding lines)


⚠️ These values must be adjusted for your own camera and cube size.

▶️ Usage

Run the script:

python detect_cubes.py


Controls:

Press Q to quit the program

🔍 Code Logic

YOLO Model

Custom trained model is loaded:

model = YOLO('my_model.pt')


Cube Detection

For each cube, bounding box and center (x, y) are extracted

Distance is calculated:

𝑍
𝑐
𝑚
=
𝐹
𝑜
𝑐
𝑎
𝑙
 
𝐿
𝑒
𝑛
𝑔
𝑡
ℎ
×
𝑅
𝑒
𝑎
𝑙
 
𝐻
𝑒
𝑖
𝑔
ℎ
𝑡
𝐵
𝑜
𝑢
𝑛
𝑑
𝑖
𝑛
𝑔
 
𝐵
𝑜
𝑥
 
𝐻
𝑒
𝑖
𝑔
ℎ
𝑡
Z
cm
	​

=
Bounding Box Height
Focal Length×Real Height
	​


Decision Making

Find the closest cube

If closest is green_box → direction = ⬅ LEFT

If closest is red_box → direction = RIGHT ➡

Visualization

Bounding boxes with labels & distance

Vertical guiding lines for alignment

Direction text at the top

📷 Example Output

Cube detection with bounding box and distance

Center markers (x,y)

Vertical alignment lines

Navigation hint

![Demo](your_screenshot.png)






