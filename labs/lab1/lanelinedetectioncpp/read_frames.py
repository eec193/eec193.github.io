import cv2
import numpy as np
import time

# Create a VideoCapture object
cap = cv2.VideoCapture(1)

# Check if camera opened successfully
if (cap.isOpened() == False):
  print("Unable to read camera feed")

# Default resolutions of the frame are obtained.The default resolutions are system dependent.
# We convert the resolutions from float to integer.
#frame_width = int(cap.get(3))
#frame_height = int(cap.get(4))
mtx = np.array([[1.5117559674732419e+02, 0., 1.6880210579629872e+02],
                [0., 1.1365411433773981e+02, 9.1959286580891799e+01],
                [0., 0., 1.]])
dist = np.array([-5.3130681063895990e-01, 2.9994013492798921e-01,
 -1.9918611960412641e-03, -3.6076825076113098e-03,
 -7.5771897951689232e-02])
count = 0
while(True):
  ret, frame = cap.read()


  if ret == True:
    frame = cv2.resize(frame, (320,180))
    frame = cv2.undistort(frame, mtx, dist, mtx)
    # Write the frame into the file 'output.avi'
    #out.write(frame)
    #cv2.imwrite("checkerboard_frames/frame%d.png" % count, frame)
    count += 1
    # Display the resulting frame
    cv2.imshow('frame',frame)

    # Press Q on keyboard to stop recording
    if cv2.waitKey(1) & 0xFF == ord('q'):
      break

  # Break the loop
  else:
    break

# When everything done, release the video capture and video write objects
cap.release()
# Closes all the frames
cv2.destroyAllWindows()
