from ultralytics import YOLO
import cv2

# Модель
model = YOLO('/home/farad/Desktop/red-green/my_model/my_model.pt')

# Калибровка камеры
FOCAL_LENGTH = 846.0    # px
REAL_HEIGHT  = 10.0     # см (высота куба)
LINE_DIST_CM = 20.0     # расстояние между зелёными линиями (см)

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    h_img, w_img = frame.shape[:2]
    x_center = w_img // 2  # центр камеры

    # Базовые линии (при Z=50 см)
    Z_ref = 50.0  
    half_span_px = (LINE_DIST_CM/2 * FOCAL_LENGTH) / Z_ref

    left_line_x  = int(x_center - half_span_px)
    right_line_x = int(x_center + half_span_px)

    # YOLO
    results = model(frame, conf=0.5)
    annotated_frame = results[0].plot()

    # ---------- Выбираем куб с минимальным расстоянием ----------
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

        # -------- ДИСТАНЦИЯ ПО ГЛУБИНЕ --------
        Z_cm = (FOCAL_LENGTH * REAL_HEIGHT) / float(height)

        # ---------- КЛАСС ОБЪЕКТА ----------
        cls_id = int(box.cls[0].item())
        class_name = results[0].names[cls_id]

        # --- Проверяем минимальную дистанцию ---
        if Z_cm < min_dist:
            min_dist = Z_cm
            closest_obj = class_name

        # --- ВИЗУАЛИЗАЦИЯ каждого куба ---
        cv2.putText(annotated_frame, f"{class_name}", (x1, max(0, y1-30)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)
        cv2.putText(annotated_frame, f"D:{Z_cm:.1f}cm", (x1, max(0, y1-10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)
        cv2.drawMarker(annotated_frame, (cx, cy), (0,0,255),
                       cv2.MARKER_CROSS, 20, 2)
        cv2.putText(annotated_frame, f"({cx},{cy})", (cx+10, cy),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200,200,0), 2)

    # --- Итоговое направление по ближайшему кубу ---
    if closest_obj == "green_box":
        direction_text = "⬅ LEFT"
    elif closest_obj == "red_box":
        direction_text = "RIGHT ➡"

    # Вертикальные линии
    cv2.line(annotated_frame, (left_line_x, 0), (left_line_x, h_img), (0,255,0), 2)
    cv2.line(annotated_frame, (right_line_x, 0), (right_line_x, h_img), (0,255,0), 2)
    cv2.line(annotated_frame, (x_center, 0), (x_center, h_img), (0,0,255), 2)

    # Стрелка/подсказка сверху
    if direction_text != "":
        cv2.putText(annotated_frame, direction_text, (50, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 200, 255), 3)

    cv2.imshow("YOLO Cubes + Distances", annotated_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
