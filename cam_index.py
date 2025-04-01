import cv2
print([i for i in range(10) if cv2.VideoCapture(i).isOpened()])
