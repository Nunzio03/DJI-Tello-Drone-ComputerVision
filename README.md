# DJI-Tello-Drone-ComputerVision
This is a ComputerVision/Control algorithm which forces a DJI Tello drone to center the camera according to your face position.

# Dependencies 
This algorithm uses : <br>
djitellopy ver 1.5 by https://github.com/damiafuentes
opencv-python ver 4.1.0.25 

install with: <br>
bash: <b> pip3 install -r requirements.txt </b>
# Usage 
run it with : <br>
bash: <b> python3 FaceDetection.py haarcascade_frontalface_default.xml </b>

# Limitations

it still has some limitations such as slow movements, setpoint oscillations and sometimes it detects faces that doesn't exist.
This is a problem because the algorithm adjust its movement according to every face in the frame

# TO DO

i'm working in : <br>
-Adding a PID algorithm for position control, in order to limit oscillation and adjust drone speed <br>
-Adding a face tracking with ID assignment, in order to follow just one given face 
