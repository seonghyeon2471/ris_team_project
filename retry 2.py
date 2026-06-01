import cv2
import numpy as np

cap = cv2.VideoCapture(0)

BOX_SIZE = 40

while True:

    ret, frame = cap.read()

    if not ret:
        continue

    h, w, _ = frame.shape

    cx = w // 2
    cy = h // 2

    x1 = cx - BOX_SIZE//2
    y1 = cy - BOX_SIZE//2

    x2 = cx + BOX_SIZE//2
    y2 = cy + BOX_SIZE//2

    roi = frame[y1:y2, x1:x2]

    # 평균 BGR
    avg_bgr = np.mean(
        roi,
        axis=(0,1)
    ).astype(int)

    # HSV 변환
    roi_hsv = cv2.cvtColor(
        roi,
        cv2.COLOR_BGR2HSV
    )

    avg_hsv = np.mean(
        roi_hsv,
        axis=(0,1)
    ).astype(int)

    b,g,r = avg_bgr
    h_,s,v = avg_hsv

    text1 = f"BGR: {b},{g},{r}"
    text2 = f"HSV: {h_},{s},{v}"

    cv2.rectangle(
        frame,
        (x1,y1),
        (x2,y2),
        (0,255,0),
        2
    )

    cv2.putText(
        frame,
        text1,
        (10,30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0,255,0),
        2
    )

    cv2.putText(
        frame,
        text2,
        (10,60),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0,255,0),
        2
    )

    cv2.imshow(
        "Color Picker",
        frame
    )

    key = cv2.waitKey(1)

    if key == ord('s'):

        print(
            "\n====== SAVED ======"
        )

        print(
            f"BGR = {avg_bgr}"
        )

        print(
            f"RGB = [{r},{g},{b}]"
        )

        print(
            f"HSV = {avg_hsv}"
        )

    elif key == ord('q'):

        break

cap.release()

cv2.destroyAllWindows()
