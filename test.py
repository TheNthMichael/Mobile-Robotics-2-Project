import cv2
import numpy as np

# Initialize variables for storing previous and current frames
prev_frame = None
curr_frame = None

# Initialize variable for storing the camera pose (position and orientation)
camera_pose = np.eye(4)

# Initialize the video capture object
cap = cv2.VideoCapture(0)

while True:
    # Capture and pre-process the current frame
    ret, frame = cap.read()
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    frame = cv2.GaussianBlur(frame, (5, 5), 0)

    # If this is the first frame, store it and continue to the next iteration
    if prev_frame is None:
        prev_frame = frame
        continue

    # Compute the optical flow between the previous and current frames
    flow = cv2.calcOpticalFlowFarneback(prev_frame, frame, None, 0.5, 3, 15, 3, 5, 1.2, 0)
    flow_mag = np.sqrt(flow[:,:,0]**2 + flow[:,:,1]**2)
    flow_mag = flow_mag.astype(np.float32)

    # Create a mask image with 255 for valid pixels and 0 for invalid pixels
    mask = np.ones_like(flow_mag, dtype=np.uint8) * 255
    mask[flow_mag == 0] = 0

    # Convert the mask image to a cv2.UMat
    mask = cv2.UMat.get(mask)

    # Fill in any invalid pixels in the flow image using the cv2.inpaint function
    flow_mag = cv2.inpaint(flow_mag, mask, 3, cv2.INPAINT_NS)

    # Compute the camera pose using the optical flow and the camera intrinsic parameters
    fx = fy = 500  # assume a fixed focal length of 500 pixels
    cx = cy = frame.shape[1] // 2  # assume the optical center is in the center of the image
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
    flow_xyz, _ = cv2.reprojectImageTo3D(flow_mag, K, mask=np.ones_like(flow_mag))
    R, t, _ = cv2.decomposeHomographyMat(np.eye(3) + flow_xyz, K)
    camera_pose = np.dot(camera_pose, np.hstack((R, t)))

    # Visualize the camera pose
    axis_length = 0.1  # length of the camera coordinate axes in meters
    imgpts, _ = cv2.projectPoints(np.float32([[0, 0, 0], [axis_length, 0, 0], [0, axis_length, 0], [0, 0, axis_length]]), camera_pose[:3, :3], camera_pose[:3, 3], K, np.zeros((1, 4)))
    frame = cv2.line(frame, tuple(imgpts[0][0]), tuple(imgpts[1][0]), (255, 0, 0), 5)
    frame = cv2.line(frame, tuple(imgpts[0][0]), tuple(imgpts[2][0]), (0, 255, 0), 5)
    frame = cv2.line(frame, tuple(imgpts[0][0]), tuple(imgpts[3][0]), (0, 0, 255), 5)

    # Display the current frame with the visualized camera pose
    cv2.imshow('Visual Odometry', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
# Release the video capture object
cap.release()
cv2.destroyAllWindows()