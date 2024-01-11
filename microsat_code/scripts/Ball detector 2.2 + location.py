import cv2
import numpy as np

# Callback function for trackbar changes
def on_trackbar(val):
    pass

# Create a window with trackbars to adjust color range
cv2.namedWindow("Color Range Adjustments")
cv2.createTrackbar("Hue Lower", "Color Range Adjustments", 10, 180, on_trackbar)
cv2.createTrackbar("Saturation Lower", "Color Range Adjustments", 100, 255, on_trackbar)
cv2.createTrackbar("Value Lower", "Color Range Adjustments", 100, 255, on_trackbar)
cv2.createTrackbar("Hue Upper", "Color Range Adjustments", 20, 180, on_trackbar)
cv2.createTrackbar("Saturation Upper", "Color Range Adjustments", 255, 255, on_trackbar)
cv2.createTrackbar("Value Upper", "Color Range Adjustments", 255, 255, on_trackbar)

def detect_ball(frame, lower_color, upper_color):
    # Convert the frame to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Threshold the image to get only the ball color
    mask = cv2.inRange(hsv, lower_color, upper_color)

    # Apply the mask to the original frame
    result = cv2.bitwise_and(frame, frame, mask=mask)

    return result, mask

def detect_sphere(frame):
    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Apply GaussianBlur to reduce noise and help circle detection
    blurred = cv2.GaussianBlur(gray, (9, 9), 2)

    # Use HoughCircles to detect circles in the image
    circles = cv2.HoughCircles(
        blurred,
        cv2.HOUGH_GRADIENT,
        dp=1,
        minDist=50,
        param1=50,
        param2=30,
        minRadius=30,
        maxRadius=100
    )

    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            # Draw the outer circle
            cv2.circle(frame, (i[0], i[1]), i[2], (0, 255, 0), 2)
            # Draw the center of the circle
            cv2.circle(frame, (i[0], i[1]), 2, (0, 0, 255), 3)

            # Print the 2D image coordinates (x, y) and radius
            print(f"2D Coordinates (x, y): ({i[0]}, {i[1]}), Radius: {i[2]}")

    return frame

def get_balloon_loc(frame, detection_settings):

    # Define the color range for the ball
    lower_color = np.array([detection_settings["hue_lower"], detection_settings["saturation_lower"], detection_settings["value_lower"]])
    upper_color = np.array([detection_settings[ "hue_upper"], detection_settings["saturation_upper"], detection_settings["value_upper"]])

    # Detect the ball and make the rest of the frame black
    result_frame, mask = detect_ball(frame, lower_color, upper_color)

    # Detect the sphere using HoughCircles and draw circles
    result_frame_with_circles = detect_sphere(result_frame)

    # Display the frame with the detected object and projected circles
    cv2.imshow("Object Detection", result_frame_with_circles)

    # Break the loop if 'q' is pressed

    # Release the webcam and close the window

if __name__ == "__main__":
    cap = cv2.VideoCapture(0)

    while True:
        hue_lower = 17 #cv2.getTrackbarPos("Hue Lower", "Color Range Adjustments")
        saturation_lower = 97 #cv2.getTrackbarPos("Saturation Lower", "Color Range Adjustments")
        value_lower = 0 #cv2.getTrackbarPos("Value Lower", "Color Range Adjustments")
        hue_upper = 55 #cv2.getTrackbarPos("Hue Upper", "Color Range Adjustments")
        saturation_upper = 255 #cv2.getTrackbarPos("Saturation Upper", "Color Range Adjustments")
        value_upper = 255 #cv2.getTrackbarPos("Value Upper", "Color Range Adjustments")

        detection_settings = {
            "hue_lower": hue_lower,
            "saturation_lower": saturation_lower,
            "value_lower": value_lower,
            "hue_upper": hue_upper,
            "saturation_upper": saturation_upper,
            "value_upper": value_upper
        }
        ret, frame = cap.read()
        get_balloon_loc(frame, detection_settings)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


