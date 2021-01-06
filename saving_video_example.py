import cv2
from datetime import datetime
#Capture video from webcam
vid_capture = cv2.VideoCapture(0)
vid_cod = cv2.VideoWriter_fourcc(*'mp4v')
filename = "videos/" + datetime.now().strftime("%Y-%m-%d-%H-%M-%S") + ".mp4"
print(filename)
output = cv2.VideoWriter(filename, vid_cod, 20.0, (640,480))
while(True):
     # Capture each frame of webcam video
     ret,frame = vid_capture.read()
     frame = cv2.flip(frame, -1)
     cv2.imshow("My cam video", frame)

     output.write(frame)
     # Close and break the loop after pressing "x" key
     if cv2.waitKey(1) &0XFF == ord('x'):
         break
# close the already opened camera
vid_capture.release()
# close the already opened file
output.release()
# close the window and de-allocate any associated memory usage
cv2.destroyAllWindows()