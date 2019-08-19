import cv2
from djitellopy import Tello
from FaceTrackerSupport import FacePointer
import time
import threading
from drawnow import *
import matplotlib.pyplot as plt


SET_POINT_X = 960/2
SET_POINT_Y = 720/2

EXPIRATION_TIME = 2  # maximum time in which a face is kept in the register if it is not in a frame

# PID constants, tuning needed
Kpx = 0
Kix = 0
Kdx = 0

Kpy = 0
Kiy = 0
Kdy = 0

DELAY = 0.02
start_time = 0

xVal = []
xTime = []


previous_error_x = 0
integral_x = 0
previous_error_y = 0
integral_y = 0


class PlotMeasurements (threading.Thread):
    def __init__(self, nome):
        threading.Thread.__init__(self)
        self.nome = nome
        plt.ion()

    def plot(self):
        global xTime, xVal, DELAY
        plt.plot(xTime, xVal)

    def run(self):

        while True:

            drawnow(self.plot)


threadPlot = PlotMeasurements("plot measurements")
threadPlot.start()


cascPath = sys.argv[1]  # Path of the model used to reveal faces
faceCascade = cv2.CascadeClassifier(cascPath)
drone = Tello()  # declaring drone object
drone.connect()
drone.takeoff()


drone.streamon()  # start camera streaming


# video_capture = cv2.VideoCapture("udp://0.0.0.0:11111")  # raw video from drone streaming address
# video_capture = cv2.VideoCapture("rtsp://192.168.1.1")  #raw video from action cam Apeman
# video_capture = cv2.VideoCapture(0)  # raw video from webcam

faceRegister = dict()
actualFaces = dict()
newActualFaces = dict()

last_expiration_time = time.time()

idCounter = 0


while True:

    # loop through frames
    # ret, frame = video_capture.read()  # used to collect frame from alternative video streams... debug purposes

    frame = drone.get_frame_read().frame  # capturing frame from drone
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # turning image into gray scale

    faces = faceCascade.detectMultiScale(  # face detection
        gray,
        scaleFactor=1.1,
        minNeighbors=20,
        minSize=(30, 30),
        flags=cv2.CASCADE_SCALE_IMAGE
    )
    actualFaces = dict()
    # Decorating image for debug purposes and looping through every detected face
    for (x, y, w, h) in faces:
        if idCounter > 99:
            idCounter = 0

        if idCounter in actualFaces or idCounter in faceRegister:
            idCounter = idCounter + 1

        actualFaces[idCounter] = FacePointer(idCounter, x, y, w, h)

        newActualFaces = actualFaces.copy()

    for actFace in actualFaces:
        minDist = 50
        faceObjID = actualFaces[actFace].ID
        for f in faceRegister:
            dist = actualFaces[actFace].distance_from_face(faceRegister[f])
            if dist < minDist:
                minDist = dist
                faceObjID = faceRegister[f].ID
        if faceObjID in faceRegister:
            faceRegister.pop(faceObjID)
        actualFaces[actFace].ID = faceObjID
        newActualFaces[faceObjID] = actualFaces[actFace]

    if time.time()-last_expiration_time > EXPIRATION_TIME:
        last_expiration_time = time.time()

        newActualFaces = dict()

    for actFace in newActualFaces:
        faceRegister[actFace] = newActualFaces[actFace]
        x = newActualFaces[actFace].x
        y = newActualFaces[actFace].y
        w = newActualFaces[actFace].w
        h = newActualFaces[actFace].h

        cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 5)  # contour rectangle
        cv2.circle(frame, (int(x+w/2), int(y+h/2)), 12, (255, 0, 0), 1)  # face-centered circle
        # print(frame.shape)
        # cv2.line(frame, (int(x+w/2), int(720/2)), (int(960/2), int(720/2)), (0, 255, 255))
        cv2.circle(frame, (int(SET_POINT_X), int(SET_POINT_Y)), 12, (255, 255, 0), 8)  # setpoint circle
        cv2.putText(frame, str(newActualFaces[actFace].ID), (x, y), 1, 2, (0, 0, 255))

        xTime.append(time.time())
        xVal.append(y + h / 2)

        """
        
        
        Drone control section
        
        
        
        """
        delay_pid = time.time() - start_time
        start_time = time.time()
        error_x = (x + w / 2) - SET_POINT_X
        error_y = SET_POINT_Y - (y + w / 2)
        integral_x = integral_x + error_x * delay_pid
        integral_y = integral_y + error_y * delay_pid

        if integral_x > 100:
            integral_x = 100
        elif integral_x < -100:
            integral_x = -100
        if integral_y > 100:
            integral_y = 100
        elif integral_y < -100:
            integral_y = - 100

        derivative_x = (error_x - previous_error_x) / delay_pid
        derivative_y = (error_y - previous_error_y) / delay_pid
        output_x = Kpx * error_x + Kix * integral_x + Kdx * derivative_x
        output_y = Kpy * error_y + Kiy * integral_y + Kdy * derivative_y
        bounded_output_x = int(100 * output_x / SET_POINT_X)
        bounded_output_y = int(100 * output_y / SET_POINT_Y)
        # outx:setpoint = mapx : 100
        previous_error_x = error_x
        previous_error_y = error_y
        # print("output X : ", bounded_output_x)
        # print("output Y : ", bounded_output_y)
        print(Kpy)
        drone.send_rc_control(bounded_output_x, 0, bounded_output_y, 0)

        time.sleep(DELAY)

    cv2.imshow('Video', frame)  # mostra il frame sul display del pc

    if cv2.waitKey(1) & 0xFF == ord('q'):  # quit from script
        # drone.land()
        # print(drone.get_battery())
        Kpy = Kpy + 0.05

        # break


# rilascio risorse
# video_capture.release()
cv2.destroyAllWindows()
