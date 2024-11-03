import cv2
import numpy as np

def extract_rotated_roi_with_fixed_origin(image, origin, yaw, side_length):
    # Half the side length of the square
    half_side = side_length / 2

    # Define the square corners with respect to the origin at the middle of the left edge
    corners = np.array([
        [0, 0],                   # Middle of the left edge (origin)
        [side_length, -half_side],   # Top-right corner
        [side_length, half_side],    # Bottom-right corner
        [0, side_length]           # Bottom-left corner
    ])

    # Create rotation matrix based on yaw
    rotation_matrix = np.array([
        [np.cos(yaw), -np.sin(yaw)],
        [np.sin(yaw), np.cos(yaw)]
    ])

    # Rotate each corner around the origin
    rotated_corners = np.dot(corners, rotation_matrix.T) + origin

    # Define destination points for perspective warp (fixed size based on side length)
    dst_points = np.array([
        [0, 0],
        [side_length - 1, 0],
        [side_length - 1, side_length - 1],
        [0, side_length - 1]
    ], dtype=np.float32)

    # Calculate perspective transform matrix
    matrix = cv2.getPerspectiveTransform(rotated_corners.astype(np.float32), dst_points)

    # Perform warp to get a straightened ROI
    roi = cv2.warpPerspective(image, matrix, (side_length, side_length))

    return roi

# Example usage
image = cv2.imread("/home/sedrica/ros2ppf_26_10/src/extras/track/my_map_new.png")
origin = (249, 0)    # Set origin to the middle of the left edge of the square
yaw = np.radians(0)    # Rotate 45 degrees
side_length = 500       # Length of each side of the square

roi = extract_rotated_roi_with_fixed_origin(image, origin, yaw, side_length)
cv2.imshow("Rotated ROI with Fixed Origin on Edge", roi)
cv2.waitKey(0)
cv2.destroyAllWindows()
