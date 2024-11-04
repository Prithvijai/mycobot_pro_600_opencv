import cv2
import numpy as np
import cv2.aruco as aruco

# Parameters
aruco_dictionary_name = aruco.DICT_6X6_250  # Change this to your desired ArUco dictionary
aruco_marker_side_length = 0.1  # Length of marker's side in meters
camera_calibration_parameters_filename = 'camera_calibration.yaml'  # Path to your camera calibration file

def euler_from_quaternion(x, y, z, w):
    """Convert quaternion to Euler angles (roll, pitch, yaw)."""
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if np.abs(sinp) >= 1:
        pitch = np.copysign(np.pi / 2, sinp)  # Use 90 degrees if out of range
    else:
        pitch = np.arcsin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

def main():
    # Load camera calibration parameters
    cv_file = cv2.FileStorage(camera_calibration_parameters_filename, cv2.FILE_STORAGE_READ)
    mtx = cv_file.getNode('K').mat()
    dist = cv_file.getNode('D').mat()
    cv_file.release()

    # Initialize webcam
    cap = cv2.VideoCapture(0)

    # Define the ArUco dictionary
    aruco_dict = aruco.getPredefinedDictionary(aruco_dictionary_name)
    parameters = aruco.DetectorParameters_create()

    while True:
        # Capture frame from webcam
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame from webcam.")
            break

        # Convert frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect ArUco markers in the frame
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if ids is not None:
            # Estimate pose for each marker detected
            for i in range(len(ids)):
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners[i], aruco_marker_side_length, mtx, dist)

                # Draw the marker and axis
                aruco.drawDetectedMarkers(frame, corners, ids)
                aruco.drawAxis(frame, mtx, dist, rvec, tvec, 0.1)  # Draw axis of length 0.1m

                # Convert rotation vector to Euler angles
                roll, pitch, yaw = euler_from_quaternion(rvec[0][0][0], rvec[0][0][1], rvec[0][0][2], rvec[0][0][3])
                print(f"Marker ID: {ids[i][0]}, Position: {tvec}, Orientation (r,p,y): {roll}, {pitch}, {yaw}")

        # Show the frame with markers
        cv2.imshow("Frame", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
