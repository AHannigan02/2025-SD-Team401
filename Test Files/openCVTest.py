import cv2
import numpy as np

cap = cv2.VideoCapture(1)  # Probe may appear as a webcam

#cap.set(cv2.CAP_PROP_FRAME_WIDTH,640)
#cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

while True:
    ret, frame = cap.read()

    cv2.imshow("Butterfly iQ3", frame)

    if cv2.waitKey(1) and 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()