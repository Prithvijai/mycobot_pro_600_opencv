import cv2

cap = cv2.VideoCapture(0)  # Change index to 0 or 1 if needed

if not cap.isOpened():
    print("Error: Could not open video.")
    exit()

num = 0
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

while True:
    succes, img = cap.read()
    if not succes:
        print("Failed to grab frame")
        break

    # Check the shape of the image
    if img is not None:
        print(img.shape)  # Check shape to ensure it's color (should be (height, width, 3))

    k = cv2.waitKey(5)
    if k == 27:  # Escape key to exit
        break
    elif k == ord('s'):  # 's' key to save the image
        cv2.imwrite('/home/kris/mycobot_pro_600_ws/src/aruco_ros/scripts/images/img' + str(num) + '.png', img)
        print("Image saved!")
        num += 1

    cv2.imshow('Img', img)

# Release and destroy all windows before termination
cap.release()
cv2.destroyAllWindows()
