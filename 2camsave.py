from datetime import datetime
import cv2
cap0 = cv2.VideoCapture(0)
cap0.set(3,640)
cap0.set(4,480)
cap1 = cv2.VideoCapture(1)
cap1.set(3,640)
cap1.set(4,480)

#Capture video from webcam
cap0 = cv2.VideoCapture(0)
vid_cod = cv2.VideoWriter_fourcc(*'mp4v')
filename1 = "videos/camdepan" + datetime.now().strftime("%Y-%m-%d-%H-%M-%S") + ".mp4"
filename2 = "videos/cambawah" + datetime.now().strftime("%Y-%m-%d-%H-%M-%S") + ".mp4"
output1 = cv2.VideoWriter(filename1, vid_cod, 20.0, (640,480))
output2 = cv2.VideoWriter(filename2, vid_cod, 20.0, (640,480))
while(True):
     # Capture each frame of webcam video
     ret0, frame0 = cap0.read()
     ret1, frame1 = cap1.read()
     cv2.imshow("My cam depan", frame0)
     cv2.imshow("My cam dbawah", frame1)
     output1.write(frame0)
     output2.write(frame1)
     # Close and break the loop after pressing "x" key
     if cv2.waitKey(1) &0XFF == ord('x'):
         break
# close the already opened camera
cap0.release()
cap1.release()
# close the already opened file
output1.release()
output2.release()
# close the window and de-allocate any associated memory usage
cv2.destroyAllWindows()