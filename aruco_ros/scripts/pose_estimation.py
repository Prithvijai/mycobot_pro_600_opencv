import cv2
import numpy as np
import yaml

def load_camera_parameters(file_path):
    with open(file_path, 'r') as f:
        params = yaml.safe_load(f)
    
    # Extract the camera matrix and distortion coefficients
    mtx = np.array(params['camera_matrix']['data']).reshape((params['camera_matrix']['rows'], params['camera_matrix']['cols']))
    dst = np.array(params['distortion_coefficients']['data']).flatten()
    
    return mtx, dst

def main():
    print("Welcome to the ArUco Marker Pose Estimator!")
    print("This program:")
    print("  - Estimates the pose of an ArUco Marker")

    # Load camera parameters
    mtx, dst = load_camera_parameters('/home/kris/mycobot_pro_600_ws/src/aruco_ros/scripts/calibration_chessboard.yaml')  # Ensure this file exists

    # You would typically add your ArUco marker detection and pose estimation logic here.
    # For example:
    # 1. Capture an image from the camera.
    # 2. Detect ArUco markers in the image.
    # 3. Estimate their poses using the camera matrix and distortion coefficients.

    # Example of using OpenCV to capture an image (ensure your camera index is correct)
    cap = cv2.VideoCapture(0)  # Change to your camera index if needed
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to capture image")
            break
        
        # Add your marker detection logic here
        # ...

        cv2.imshow("Frame", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
