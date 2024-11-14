import cv2
import os
import numpy as np
import cv2.aruco as aruco

# Create a directory for saving pose images if it doesn't exist
pose_images_dir = '/home/kris/mycobot_pro_600_ws/src/aruco_ros/scripts/pose_images'
if not os.path.exists(pose_images_dir):
    os.makedirs(pose_images_dir)

# Initialize a counter for image numbering
image_counter = 1

# Camera matrix and distortion coefficients (replace these with your calibration values)
camera_matrix = np.array([[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]])  # Example values
dist_coeffs = np.zeros((4, 1))  # Example values

# Camera settings
cap = cv2.VideoCapture(0)  # Adjust the index based on your camera

# ArUco marker parameters
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
parameters = aruco.DetectorParameters_create()

# Desired width and height for the output feed
desired_width = 640
desired_height = 480

while True:
    # Read a frame from the video feed
    ret, frame = cap.read()
    if not ret:
        print("Failed to capture image")
        break

    # Resize the frame
    frame = cv2.resize(frame, (desired_width, desired_height))

    # Convert the frame to grayscale for marker detection
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect markers in the frame
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    # Draw the detected markers
    aruco.drawDetectedMarkers(frame, corners, ids)

    # Display the frame with markers
    cv2.imshow('Video Feed', frame)

    # Wait for key press
    key = cv2.waitKey(1) & 0xFF

    # If 's' is pressed, capture the image and save
    if key == ord('s'):
        # Create a filename with the current counter value
        image_filename = os.path.join(pose_images_dir, f'image{image_counter}.jpg')
        cv2.imwrite(image_filename, frame)
        print(f"Image saved as {image_filename}")

        # Check for markers again in the current frame for pose estimation
        if ids is not None:
            for marker_id, corner in zip(ids.flatten(), corners):
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corner, 0.1, camera_matrix, dist_coeffs)

                # Output pose estimation
                position = tvec[0].flatten()
                orientation = rvec[0].flatten()

                # Print pose estimates in the terminal for this saved image
                print(f"Marker ID: {marker_id}, Position: {position}, Orientation: {orientation}")

        image_counter += 1  # Increment the counter

    # If 'q' is pressed, exit the loop
    if key == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()
