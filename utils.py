from djitellopy import Tello
import cv2
import numpy as np


# Function to initialie the drone
def initializeTello():
    myDrone = Tello()
    myDrone.connect()
    myDrone.for_back_velocity = 0  # forward and backward
    myDrone.left_right_velocity = 0  # left and  right
    myDrone.up_down_velocity = 0  # up and down
    myDrone.yaw_velocity = 0  # rotation
    myDrone.speed = 0  # general speed

    # get battery status
    print(myDrone.get_battery())  # to know how much charge is left
    myDrone.streamoff()  # first turn off and turn on
    myDrone.streamon()
    return myDrone


# function to get frames
def telloGetFrame(myDrone, w=360, h=240):
    myFrame = myDrone.get_frame_read()  # to get the image from the drone
    myFrame = myFrame.frame
    img = cv2.resize(myFrame, (w, h))  # fame can be resized as per requirements
    return img


# function to find faces
def findFace(img):
    faceCascade = cv2.CascadeClassifier("haarcascade_frontalface_default.xml")  # opencv
    imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)  # grey scale image
    faces = faceCascade.detectMultiScale(
        imgGray, 1.1, 6
    )  # find faces, scale-factors and minimum neighbours - change as per the requirements

    myFaceListC = []  # center x
    myFaceListArea = []  # center y

    for x, y, w, h in faces:
        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)  # draw rectangle
        cx = x + w // 2
        cy = y + h // 2
        area = w * h
        myFaceListArea.append(area)
        myFaceListC.append([cx, cy])

    # check if any faces are visible or not
    if len(myFaceListArea) != 0:
        i = myFaceListArea.index(
            max(myFaceListArea)
        )  # find the index of max posiiton in the area, or closed face to the drone
        return img, [myFaceListC[i], myFaceListArea[i]]
    else:
        return img, [[0, 0], 0]


def trackFace(myDrone, info, w, pid, pError):
    """
    Function to track face and follow.
    Fucntion needs : Drone Object, info - receive from face, width, PID and error
    find the error between actual value and where it should be
    (actual value = width / 2)

    for ex:
        width = 640 , then actual_value == 320, means our position should always be 320 if we are tracking object properly
        if value > 320 POSITIVE ERROR
            value < 320 then NEGATIVE ERROR

        the value 'cx' is subtracted form 320, this will giev us error, here cx is info[0][0]


    PID controller - helps for smoothning the transitions of drone movements, when teh drone is moving it should move smoothly
    in PID we are controlling speed

    pError= previous error

    """

    ## PID
    error = info[0][0] - w // 2
    speed = pid[0] * error + pid[1] * (error - pError)
    speed = int(np.clip(speed, -100, 100))  # clipping the speed between -100 to 100

    print(speed)
    if info[0][0] != 0:  # detecting the face or not
        myDrone.yaw_velocity = speed
    else:
        # set the default values
        myDrone.for_back_velocity = 0
        myDrone.left_right_velocity = 0
        myDrone.up_down_velocity = 0
        myDrone.yaw_velocity = 0
        error = 0

    # Sending the values to the drone
    if myDrone.send_rc_control:
        myDrone.send_rc_control(
            myDrone.left_right_velocity,
            myDrone.for_back_velocity,
            myDrone.up_down_velocity,
            myDrone.yaw_velocity,
        )
    return error
