import cv2

# URL of the DroidCam video feed
url = "http://192.168.0.109:5050/video"

cap = cv2.VideoCapture(url)

if not cap.isOpened():
    print("Error: Couldn't open video stream.")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Couldn't receive frame.")
        break

    cv2.imshow("DroidCam Feed", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()