import cv2
import numpy as np
import glob

# Chessboard dimensions and square size in meters
nX, nY, square_size = 9, 6, 0.025  # Adjust nX, nY for inner corners

# 3D point coordinates
obj_pts_3D = np.zeros((nX * nY, 3), np.float32)
obj_pts_3D[:, :2] = np.mgrid[0:nX, 0:nY].T.reshape(-1, 2) * square_size

obj_points, img_points = [], []  # Points in 3D and 2D space

for img_file in glob.glob("/home/kris/mycobot_pro_600_ws/src/aruco_ros/scripts/images/*.png"):
    img = cv2.imread(img_file)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    found, corners = cv2.findChessboardCorners(gray, (nX, nY), None)
    if found:
        obj_points.append(obj_pts_3D)
        img_points.append(corners)

ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, gray.shape[::-1], None, None)

# Save to a file
cv_file = cv2.FileStorage("/home/kris/mycobot_pro_600_ws/src/aruco_ros/scripts/calibration_chessboard.yaml", cv2.FILE_STORAGE_WRITE)
cv_file.write("K", mtx)
cv_file.write("D", dist)
cv_file.release()
