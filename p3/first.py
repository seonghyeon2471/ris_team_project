import cv2
import numpy as np

# =========================================
# CAMERA
# =========================================
cap = cv2.VideoCapture(0)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# =========================================
# HSV RANGE
# =========================================
COLOR_RANGES = {
    "RED": (
        np.array([169, 168, 96]),
        np.array([179, 222, 157])
    ),

    "YELLOW": (
        np.array([16, 137, 142]),
        np.array([30, 214, 195])
    ),

    "BLUE": (
        np.array([106, 168, 54]),
        np.array([131, 210, 82])
    )
}

# =========================================
# LOOP
# =========================================
while True:

    ret, frame = cap.read()
    if not ret:
        break

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    for color_name, (lower, upper) in COLOR_RANGES.items():

        mask = cv2.inRange(hsv, lower, upper)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        if len(contours) > 0:

            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)

            if area > 300:

                M = cv2.moments(largest)

                if M["m00"] > 0:

                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])

                    x, y, w, h = cv2.boundingRect(largest)

                    cv2.rectangle(
                        frame,
                        (x, y),
                        (x + w, y + h),
                        (0, 255, 0),
                        2
                    )

                    cv2.circle(
                        frame,
                        (cx, cy),
                        6,
                        (0, 0, 255),
                        -1
                    )

                    text = f"{color_name} ({cx},{cy})"

                    cv2.putText(
                        frame,
                        text,
                        (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (255, 255, 255),
                        2
                    )

                    print(
                        f"{color_name:<6} centroid=({cx:3d},{cy:3d}) area={int(area)}",
                        end="    "
                    )

        cv2.imshow(f"{color_name}_MASK", mask)

    print()

    cv2.imshow("FRAME", frame)

    key = cv2.waitKey(1) & 0xFF

    if key == 27:
        break

cap.release()
cv2.destroyAllWindows()
