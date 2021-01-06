import cv2
import numpy as np
import pyzbar.pyzbar as pyzbar
from datetime import datetime


cap = cv2.VideoCapture(0)
font = cv2.FONT_HERSHEY_PLAIN
vid_cod = cv2.VideoWriter_fourcc(*'mp4v')
filename = "videos/" + datetime.now().strftime("%Y-%m-%d-%H-%M-%S") + ".mp4"
output = cv2.VideoWriter(filename, vid_cod, 20.0, (640,480))

while True:
    _, frame = cap.read()
    value = ""
    decodedObjects = pyzbar.decode(frame)
    frame = cv2.flip(frame,1)
    for obj in decodedObjects:
        #print("Data", obj.data)
        cv2.putText(frame, str(obj.data), (50, 50), font, 2,
                    (255, 0, 0), 3)
        value = str(obj.data)
    if value == "b'Tai'":
        print("benar")
    else:
        print("salah")
    output.write(frame)
    cv2.imshow("Frame", frame)

    key = cv2.waitKey(1)
    if key == 27:
        break

# close the already opened camera
cap.release()
# close the already opened file
output.release()
# close the window and de-allocate any associated memory usage
cv2.destroyAllWindows()