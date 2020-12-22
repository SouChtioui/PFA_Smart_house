import os
import time

name=input('what''s your name ?\n')
picsNumber=input('how many pictures do you want to take ?\n')
picsNumber=int(picsNumber)
path="/home/pi/firebase/pi-face-recognition/dataset/{}".format(name)
try:
    os.mkdir(path)
except OSError:
    print("Error creating directory")
for i in range(picsNumber):
    s='fswebcam -r 255x255 --no-banner -S 15 --jpeg 95 --save {}/000{}.jpg'.format(path,str(i+1))
    print("taking picture number {}".format(str(i+1)))
    os.system(s)
    time.sleep(1)
