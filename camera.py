import cv2
import serial

# Open a serial connection to the Arduino
ser = serial.Serial('COM3', 9600)

# Initialize webcam
cap = cv2.VideoCapture(1)

# Get the width and height of the video input
width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

print(f"Width: {width}, Height: {height}")

# Coordinates of the region of interest (x, y, width, height)
roi_x = 450
roi_y = 0
roi_width = 150
roi_height = 250

while True:
    # Returns the images/frames from camera
    success, frame = cap.read()
    
    if not success:
        print("Camera not working!")
        break

    # Crop the frame using the defined ROI
    roi = frame[roi_y:roi_y+roi_height, roi_x:roi_x+roi_width]

    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    
    edges = cv2.Canny(blurred, 50, 150)
    
    contours, _ = cv2.findContours(edges.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Draw contours and their center points
    for contour in contours:
        M = cv2.moments(contour)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])

            # Draw the contour and center point on the frame
            cv2.drawContours(frame, [contour], -1, (0, 255, 0), 2)
            cv2.circle(frame, (cX, cY), 7, (255, 0, 0), -1)
            cv2.putText(frame, f"({roi_width - cX}, {roi_height - cY})", (cX - 50, cY - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                    
            # Send the centre coordinates to Arduino
            # cY = x and cX = y to make the camera's right edge the horizontal axis
            coord_string = f"{roi_height - cY},{roi_width - cX}\n"
            ser.write(coord_string.encode())

    # Display the frame
    cv2.imshow('Contours', frame)
    
    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

ser.close()
cap.release()
cv2.destroyAllWindows()