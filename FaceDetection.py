import cv2
import sys
from djitellopy import Tello
import time

# PID constants, tuning needed
Kpx = 0
Kix = 0
Kdx = 0
Kpy = 2
Kiy = 0
Kdy = 1

#

DELAY = 0.002
SET_POINT_X = 960 / 2
SET_POINT_Y = 720 / 2

previous_error_x = 0
integral_x = 0
previous_error_y = 0
integral_y = 0

cascPath = sys.argv[1]  # Path of the model used to reveal faces
faceCascade = cv2.CascadeClassifier(cascPath)
drone = Tello()  # declaring drone object
drone.connect()
drone.takeoff()


drone.streamon()  # start camera streaming


# video_capture = cv2.VideoCapture("udp://0.0.0.0:11111")  # raw video from drone streaming address
# video_capture = cv2.VideoCapture("rtsp://192.168.1.1")  #raw video from action cam Apeman
# video_capture = cv2.VideoCapture(0)  # raw video from webcam

while True:
    # loop through frames

    # ret, frame = video_capture.read()  # used to collect frame from alternative video streams

    frame = drone.get_frame_read().frame  # capturing frame from drone
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # turning image into gray scale

    faces = faceCascade.detectMultiScale(  # face detection
        gray,
        scaleFactor=1.1,
        minNeighbors=5,
        minSize=(30, 30),
        flags=cv2.CASCADE_SCALE_IMAGE
    )

    # Decorating image for debug purposes and looping through every detected face
    for (x, y, w, h) in faces:

        cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 5)  # contour rectangle
        cv2.circle(frame, (int(x+w/2), int(y+h/2)), 12, (255, 0, 0), 1)  # face-centered circle
        # print(frame.shape)
        # cv2.line(frame, (int(x+w/2), int(720/2)), (int(960/2), int(720/2)), (0, 255, 255))

        cv2.circle(frame, (int(SET_POINT_X), int(SET_POINT_Y)), 12, (0, 255, 255), 8)  # setpoint circle

        error_x = (x+w/2) - SET_POINT_X
        error_y = SET_POINT_Y - (y+w/2)
        integral_x = integral_x + error_x * DELAY
        integral_y = integral_y + error_x * DELAY
        derivative_x = (error_x - previous_error_x) / DELAY
        derivative_y = (error_y - previous_error_y) / DELAY
        output_x = Kpx * error_x + Kix * integral_x + Kdx * derivative_x
        output_y = Kpy * error_y + Kiy * integral_y + Kdy * derivative_y
        bounded_output_x = int(100 * output_x / SET_POINT_X)
        bounded_output_y = int(100 * output_y / SET_POINT_Y)
        # outx:240 = mapx : 100
        previous_error_x = error_x
        previous_error_y = error_y
        print("output X : ", bounded_output_x)
        print("output Y : ", bounded_output_y)
        drone.send_rc_control(bounded_output_x, 0, bounded_output_y, 0)

        time.sleep(DELAY)

    cv2.imshow('Video', frame)  # mostra il frame sul display del pc

    if cv2.waitKey(1) & 0xFF == ord('q'):  # quit from
        print(drone.get_battery())
        break

# rilascio risorse
# video_capture.release()
cv2.destroyAllWindows()
