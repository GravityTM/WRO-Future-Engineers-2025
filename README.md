Table of Contents
-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
- The Team


The Team
-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
### Banu Isgandarli

<img src="https://github.com/user-attachments/assets/d1633134-9a48-4e6a-9c2a-a42ba5db09b0" width="400px" height="500px"/>


Hello! My name is Banu, and I am 15 years old. To date, I have participated in four competitions, and this year marks my first foray into the WRO Future Engineers category. In my free time, I am passionate about robotics, programming, and playing the violin. I also enjoy exploring new concepts and continuously developing my skills. Additionally, I have a strong interest in chess and Formula 1, which reflect my enthusiasm for both strategic thinking and dynamic problem-solving.🎀🏎




### Jannat Ozdemir


<img src="https://github.com/user-attachments/assets/7d52f428-0e74-4901-9b23-cc38ac3ba7bd" width="200px" height="200px"/>

My name is Jannat Özdemir, I am 16 years old.My zodiac sign is Sagittarius. In my free time, I enjoy horse riding, reading books, and learning new languages. I also love traveling around different countries. I have previously participated in the LEGO Line category. I am passionate about cybersecurity.l can make the best desserts in the world My signature phrase is: “We will beat everyone"💅💎




### Fatima Alizade

<img src="https://github.com/user-attachments/assets/9bb84605-dea2-4183-8ffc-12d16d6cfe33" width="100px" height="200px"/>

Hello, my name is Fatima Elizade. I am 16 years old. I study in high school. I like programming. I enjoy playing games and also creating my own games. I know 3 programming languages.I love eating food.I like coconut🥥🫶🏿




3D Model of Robot
-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
| Top 3D Model | Side 3D Model | Front 3D Model | Isometric 3D Model |
|--------------|---------------|----------------|----------------|
| <img src="https://github.com/user-attachments/assets/3022cd89-6f7f-4c03-b50d-51091934c3ac" width="200"/> | <img src="https://github.com/user-attachments/assets/42a4a9e2-2a62-4ac1-b08d-6e0914a790e9" width="200"/> | <img src="https://github.com/user-attachments/assets/97e87bd3-68e6-482d-ac02-700d0cf6718d" width="200"/> | <img src="https://github.com/user-attachments/assets/02416aa0-53ec-409b-bdfa-e27084d24a1f" width="200"/> |
-----------------------------------------------------------------------
# YOLO Cube Detection & Distance Estimation

This project uses a **YOLOv8 model** and OpenCV to detect cubes (green and red), estimate their distance from the camera, and show navigation hints (`⬅ LEFT` or `RIGHT ➡`) depending on the closest cube.  
It can be used in robotics competitions such as **WRO Future Engineers**.

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

## 📷 Example Output
- Bounding boxes with labels and distances  
- Center markers `(x,y)`  
- Vertical guide lines  
- Navigation hint (`⬅ LEFT` or `RIGHT ➡`)  

Add a screenshot or video here:

```markdown
![Demo](your_screenshot.png)
```

---
