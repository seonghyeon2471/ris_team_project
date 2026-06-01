import cv2
import time

# 카메라 열기
cap = cv2.VideoCapture(0)

# MJPG 사용 (USB 카메라에서 지연 감소)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

# 해상도 설정
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# 버퍼 최소화
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

prev_time = time.time()

while True:

    # 오래된 프레임 버리기
    for _ in range(3):
        cap.grab()

    ret, frame = cap.read()

    if not ret:
        print("Camera not found.")
        break

    # FPS 계산
    current_time = time.time()
    fps = 1 / (current_time - prev_time)
    prev_time = current_time

    # FPS 표시
    cv2.putText(
        frame,
        f"FPS: {fps:.1f}",
        (20, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        1,
        (0, 255, 0),
        2
    )

    cv2.imshow("Camera Test", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
