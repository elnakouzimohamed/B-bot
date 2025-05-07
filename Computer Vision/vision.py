from picamera2 import Picamera2
from ultralytics import YOLO
import cv2
import time
from skimage.metrics import structural_similarity as ssim
import numpy as np
import os
import RPi.GPIO as GPIO

# GPIO setup
GREEN_LED = 17
RED_LED = 23  # changed from 27
BUZZER = 22

GPIO.setmode(GPIO.BCM)
GPIO.setup(GREEN_LED, GPIO.OUT)
GPIO.setup(RED_LED, GPIO.OUT)
GPIO.setup(BUZZER, GPIO.OUT)

# Load YOLOv8 model
model = YOLO("yolov8n.pt")

# Create folders
save_folder = "NeuralNetworkCamera"
reference_folder = "references"
os.makedirs(save_folder, exist_ok=True)

# Load 5 reference images
reference_images = []
reference_files = sorted(os.listdir(reference_folder))[:5]

for filename in reference_files:
    if filename.lower().endswith((".jpg", ".jpeg", ".png")):
        path = os.path.join(reference_folder, filename)
        img = cv2.imread(path)
        img = cv2.resize(img, (640, 480))
        reference_images.append(img)

print(f"[INFO] Loaded {len(reference_images)} reference images.")

# Start camera
picam2 = Picamera2()
camera_config = picam2.create_preview_configuration(main={"size": (640, 480)})
picam2.configure(camera_config)
picam2.start()
time.sleep(1)  # let camera stabilize

# Compare function
def compare_images(img1, img2):
    grayA = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
    grayB = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
    score, _ = ssim(grayA, grayB, full=True)
    return score

# Main loop
try:
    while True:
        frame = picam2.capture_array()
        frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

        results = model.predict(source=frame, conf=0.5, imgsz=320, verbose=False)[0]
        person_detected = False

        for box in results.boxes:
            label = model.names[int(box.cls[0])]
            if label == "person":
                person_detected = True
                break

        if not person_detected:
            GPIO.output(RED_LED, GPIO.HIGH)
            GPIO.output(GREEN_LED, GPIO.LOW)
            GPIO.output(BUZZER, GPIO.LOW)
            print("[INFO] No human detected.")
        else:
            GPIO.output(GREEN_LED, GPIO.HIGH)
            GPIO.output(RED_LED, GPIO.LOW)
            print("[INFO] Human detected!")

            detected_image = frame.copy()

            matched = False
            for ref_img in reference_images:
                score = compare_images(ref_img, detected_image)
                if score > 0.7:
                    matched = True
                    break

            if matched:
                print("✅ [MATCH] Reference person found! Buzzing...")
                GPIO.output(BUZZER, GPIO.HIGH)
                time.sleep(1)
                GPIO.output(BUZZER, GPIO.LOW)
            else:
                GPIO.output(BUZZER, GPIO.LOW)

        cv2.imshow("Smart Camera", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("Exiting...")

finally:
    GPIO.cleanup()
    cv2.destroyAllWindows()
