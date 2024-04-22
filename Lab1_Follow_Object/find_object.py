# Name:Huaiyuan Rao
# Function: Object Tracking
#!/usr/bin/env python
import cv2
import numpy as np

cap = cv2.VideoCapture(0)

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.medianBlur(gray,5)

    # All these are test filtering functions

    # et3,gray = cv2.threshold(gray,127,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    # gray_blurred = cv2.GaussianBlur(gray, (9, 9), 2)
    # cimg = cv2.cvtColor(gray,cv2.COLOR_GRAY2BGR)
    # kernel = np.array([[-1,-1,-1], [-1,9,-1], [-1,-1,-1]])
    # gray = cv2.filter2D(gray, -1, kernel)
    # gray = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
    # kernel = np.ones((7, 7), np.uint8) 
    # img_erosion = cv2.erode(gray, kernel, iterations=1)
    # img_dilation = cv2.dilate(gray, kernel, iterations=1)

    circles = cv2.HoughCircles(gray,cv2.HOUGH_GRADIENT,1,300,
                            param1=70,param2=60,minRadius=0,maxRadius=0)

    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0,:]:
            # draw the outer circle
            cv2.circle(frame,(i[0],i[1]),i[2],(0,255,0),2)
            # draw the center of the circle
            cv2.circle(frame,(i[0],i[1]),2,(0,0,255),3)

            # Coordinate text
            coord_text = f"({i[0]}, {i[1]})"
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(frame, coord_text, (i[0], i[1]), font, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

            # print(f"Center coordinates: (x: {i[0]}, y: {i[1]})")

    # Display the resulting frame
    cv2.imshow('find_object',frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
