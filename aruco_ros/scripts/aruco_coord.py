import cv2
import cv2.aruco as aruco
import numpy as np

# Load the image
img = cv2.imread('/home/kris/mycobot_pro_600_ws/src/aruco_ros/scripts/dataset/img1.jpeg')

# Initialize the ArUco dictionary and detector parameters
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
parameters = aruco.DetectorParameters()

# Detect the markers in the image
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
detector = aruco.ArucoDetector(aruco_dict, parameters)
corners, ids, _ = detector.detectMarkers(gray)

if ids is not None:
    print("Detected ArUco markers:")
    for i in range(len(ids)):
        print(f"Marker ID: {ids[i][0]}, Corners: {corners[i]}")
    
    # Camera calibration parameters (or use pre-calibrated values)
    focal_length = 700  # Adjust this based on your camera
    camera_matrix = np.array([[focal_length, 0, img.shape[1] / 2],
                               [0, focal_length, img.shape[0] / 2],
                               [0, 0, 1]])
    dist_coeffs = np.zeros((4, 1))  # Assume no distortion for simplicity

    # Marker side length in meters
    marker_length = 0.05  # Adjust this to your actual marker size

    # Estimate pose for each marker
    rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, dist_coeffs)

    # Draw detected markers and their axes
    aruco.drawDetectedMarkers(img, corners)

    for i in range(len(ids)):
        img = draw_axis(img, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], marker_length * 0.5)

        print(f"Marker ID: {ids[i][0]}")
        print(f"Translation vector (tvec): {tvecs[i].ravel()}")
        print(f"Rotation vector (rvec): {rvecs[i].ravel()}")

    # Show the result
    cv2.imshow("Detected ArUco markers with Pose", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

else:
    print("No ArUco markers detected.")

def draw_axis(img, camera_matrix, dist_coeffs, rvec, tvec, length=0.1):
    # Define axis points in 3D space
    axis = np.float32([[length, 0, 0],   # X axis
                        [0, length, 0],   # Y axis
                        [0, 0, -length]]).reshape(-1, 3)  # Z axis

    # Project 3D points to 2D image plane
    imgpts, _ = cv2.projectPoints(axis, rvec, tvec, camera_matrix, dist_coeffs)

    # Debugging output
    print(f"imgpts before reshape: {imgpts}")  # Debugging line

    # Ensure imgpts is a proper 2D array
    imgpts = imgpts.reshape(-1, 2)  # Reshape to (N, 2) for line drawing
    
    # Debugging output to confirm proper shapes
    print(f"imgpts after reshape: {imgpts}")

    # Draw lines for each axis
    img = cv2.line(img, tuple(imgpts[0].astype(int)), tuple(imgpts[1].astype(int)), (255, 0, 0), 5)  # X axis (red)
    img = cv2.line(img, tuple(imgpts[0].astype(int)), tuple(imgpts[2].astype(int)), (0, 255, 0), 5)  # Y axis (green)
    # Z axis is not drawn to avoid IndexError

    return img
