import cv2
import sys
from djitellopy import Tello
from FaceTrackerSupport import FacePointer
import time

TOLERANCE_X = 5
TOLERANCE_Y = 5
SLOWDOWN_THRESHOLD_X = 20
SLOWDOWN_THRESHOLD_Y = 20
DRONE_SPEED_X = 20
DRONE_SPEED_Y = 20
SET_POINT_X = 960/2
SET_POINT_Y = 720/2


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
EXPIRATION_TIME = 5
last_expiration_time = time.time()

idCounter = 0


while True:

    # loop through frames
    # ret, frame = video_capture.read()  # used to collect frame from alternative video streams

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
        actualFaces[idCounter] = FacePointer(idCounter, x, y, w, h)
        idCounter = idCounter+1
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
    print(time.time()-last_expiration_time)
    if time.time()-last_expiration_time > EXPIRATION_TIME:
        last_expiration_time = time.time()
        faceRegister = dict()

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


        """
        
        
        Drone control section
        
        
        
        """
        if actFace == 0:
            distanceX = x+w/2 - SET_POINT_X
            distanceY = y+h/2 - SET_POINT_Y

            up_down_velocity = 0
            right_left_velocity = 0

            if distanceX < -TOLERANCE_X:
                print("sposta il drone alla sua SX")
                right_left_velocity = - DRONE_SPEED_X

            elif distanceX > TOLERANCE_X:
                print("sposta il drone alla sua DX")
                right_left_velocity = DRONE_SPEED_X
            else:
                print("OK")

            if distanceY < -TOLERANCE_Y:
                print("sposta il drone in ALTO")
                up_down_velocity = DRONE_SPEED_Y
            elif distanceY > TOLERANCE_Y:
                print("sposta il drone in BASSO")
                up_down_velocity = - DRONE_SPEED_Y

            else:
                print("OK")

            if abs(distanceX) < SLOWDOWN_THRESHOLD_X:
                right_left_velocity = int(right_left_velocity / 2)
            if abs(distanceY) < SLOWDOWN_THRESHOLD_Y:
                up_down_velocity = int(up_down_velocity / 2)

            drone.send_rc_control(right_left_velocity, 0, up_down_velocity, 0)

    cv2.imshow('Video', frame)  # mostra il frame sul display del pc

    if cv2.waitKey(1) & 0xFF == ord('q'):  # quit from script
        break

# rilascio risorse
# video_capture.release()
cv2.destroyAllWindows()
