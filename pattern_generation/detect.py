# Modified version from Cheng, Minghao
import cv2

filename = "table.jpg" # change this to the file you want to test
image = cv2.imread(filename) 
print(f"Image shape: {image.shape}")

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
parameters =  cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(dictionary, parameters)

markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(image)

# print(markerIds)
print(f"Detected {len(markerIds)} markers in {filename}")

if markerIds is not None:
    cv2.aruco.drawDetectedMarkers(image, markerCorners, markerIds)

cv2.imwrite("output_with_markers.png", image)