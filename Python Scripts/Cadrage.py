import time
from imutils.video import VideoStream
from imutils.video import FPS
import imutils
import cv2

# initialize the video stream and allow the camera sensor to warm up
print("[INFO] starting video stream...")
vs = VideoStream(src=0).start()
time.sleep(2.0)

# start the FPS counter
fps = FPS().start()


while True:
    frame = vs.read()
    frame = imutils.resize(frame, width=500)


    # display the image to our screen
    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1) & 0xFF

    # if the `q` key was pressed, break from the loop
    if key == ord("q"):
        break

    # update the FPS counter
    fps.update()
# do a bit of cleanup
cv2.destroyAllWindows()
vs.stop()
