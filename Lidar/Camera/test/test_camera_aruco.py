import cv2
import cv2.aruco as aruco

# Select the dictionary of ArUco markers
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)

# Initialize video capture (0 = default webcam)
cap = cv2.VideoCapture(0)

# Create parameters object for detection
parameters = aruco.DetectorParameters()

# Create detector object
detector = aruco.ArucoDetector(aruco_dict, parameters)

print("🔍 Starting ArUco detection. Press 'q' to quit.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("❌ Failed to grab frame.")
        break

    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect markers
    corners, ids, _ = detector.detectMarkers(gray)

    # Draw detected markers
    if ids is not None:
        aruco.drawDetectedMarkers(frame, corners, ids)
        for i, marker_id in enumerate(ids):
            print(f"🆔 Marker detected: ID = {marker_id[0]}")

    # Show the result
    cv2.imshow("ArUco Detection", frame)

    # Press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Cleanup
cap.release()
cv2.destroyAllWindows()
