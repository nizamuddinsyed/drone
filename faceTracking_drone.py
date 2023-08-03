from utils import *
import cv2

w, h = 360, 240  # window size is fine
pid = [0.4, 0.4, 0]  # kP, kD and kI
pError = 0
startCounter = 0  # for takeoff set this parametr to - > 0, 1 otherwise


# Initialize the Drone
myDrone = initializeTello()

while True:
    ## Flight - Drone takeoff condition
    if startCounter == 0:
        myDrone.takeoff()
        startCounter = 1

    ## Step 1 : get the frames / img
    img = telloGetFrame(myDrone, w, h)

    ## Step 2 : find the diffferent faces from images
    img, info = findFace(img)

    ## Step 3: track the face
    pError = trackFace(myDrone, info, w, pid, pError)

    # print(info[0][0]) # cx value , center X -  rotate
    cv2.imshow("Image", img)  # show image

    # in case something goes wrong, press 'q' button on keyboard , the drone will land
    if cv2.waitKey(1) & 0xFF == ord("q"):
        myDrone.land()
        break
