import numpy as np
import cv2
import time


class ArucoDetector:
    def __init__(self, marker_size=0.05, camera_matrix=None, dist_coeffs=None):
        """
        Initialize the ArUco marker detector

        Parameters:
            marker_size: Physical size of the ArUco marker in meters (default: 0.05m or 5cm)
            camera_matrix: 3x3 camera intrinsic matrix
            dist_coeffs: Camera distortion coefficients
        """
        # Store the actual marker size in meters
        self.marker_size = marker_size
        print(f"Using marker size: {self.marker_size} meters")

        # If camera_matrix is not provided, use a default estimation
        if camera_matrix is None:
            # Default camera matrix for a 640x480 camera
            self.camera_matrix = np.array([
                [800, 0, 320],
                [0, 800, 240],
                [0, 0, 1]
            ], dtype=np.float32)
        else:
            self.camera_matrix = camera_matrix

        # If distortion coefficients are not provided, assume no distortion
        if dist_coeffs is None:
            self.dist_coeffs = np.zeros((5, 1), dtype=np.float32)
        else:
            self.dist_coeffs = dist_coeffs

        # Set up ArUco detector for OpenCV 4.11.0
        dictionary_id = cv2.aruco.DICT_4X4_250
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(dictionary_id)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

    def detect_markers(self, frame):
        """
        Detect ArUco markers in the given frame

        Parameters:
            frame: Input camera frame

        Returns:
            frame_markers: Frame with detected markers visualized
            marker_positions: Dictionary mapping marker IDs to their positions (x,y,z) in meters
        """
        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect markers
        corners, ids, rejected = self.aruco_detector.detectMarkers(gray)

        # Create a copy of the frame to draw on
        frame_markers = frame.copy()

        marker_positions = {}

        if ids is not None:
            # Draw detected markers
            cv2.aruco.drawDetectedMarkers(frame_markers, corners, ids)

            # Estimate pose for each marker
            for i in range(len(ids)):
                # Get the corner points for this marker
                marker_corners = corners[i]

                # Estimate pose with the specified marker size
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    marker_corners, self.marker_size, self.camera_matrix, self.dist_coeffs
                )

                # Get the first elements correctly based on shape
                rvec = rvecs[0]
                tvec = tvecs[0]

                # Draw axis for each marker
                cv2.drawFrameAxes(frame_markers, self.camera_matrix, self.dist_coeffs,
                                  rvec, tvec, self.marker_size / 2)

                # Store position information - correctly access translation vector elements
                marker_id = ids[i][0]
                position = {
                    'x': float(tvec[0][0]) if tvec.ndim > 1 else float(tvec[0]),  # x position in meters
                    'y': float(tvec[0][1]) if tvec.ndim > 1 else float(tvec[1]),  # y position in meters
                    'z': float(tvec[0][2]) if tvec.ndim > 1 else float(tvec[2]),  # z position in meters
                }
                position['distance'] = np.sqrt(
                    position['x'] ** 2 + position['y'] ** 2 + position['z'] ** 2)  # Euclidean distance

                marker_positions[marker_id] = position

                # Add text with distance information
                cv2.putText(frame_markers, f"ID:{marker_id} d:{position['distance']:.2f}m",
                            (int(corners[i][0][0][0]), int(corners[i][0][0][1]) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        return frame_markers, marker_positions

    def run_camera(self, camera_id=0):
        """
        Run a continuous detection loop using the specified camera

        Parameters:
            camera_id: Camera device ID (default: 0)
        """
        cap = cv2.VideoCapture(camera_id)

        if not cap.isOpened():
            print("Error: Could not open camera.")
            return

        print(f"Camera {camera_id} opened successfully.")

        try:
            while True:
                ret, frame = cap.read()

                if not ret:
                    print("Error: Couldn't read frame.")
                    break

                # Process frame
                frame_markers, positions = self.detect_markers(frame)

                # Display marker positions
                if positions:
                    print("\nDetected Markers:")
                    for marker_id, pos in positions.items():
                        print(f"Marker {marker_id}: x={pos['x']:.3f}m, y={pos['y']:.3f}m, z={pos['z']:.3f}m, " +
                              f"distance={pos['distance']:.3f}m")

                # Show the frame
                cv2.imshow('ArUco Marker Detection', frame_markers)

                # Break the loop if 'q' is pressed
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        except Exception as e:
            print(f"Error during detection: {e}")
            import traceback
            traceback.print_exc()
        finally:
            # Release resources
            cap.release()
            cv2.destroyAllWindows()


# Example usage:
if __name__ == "__main__":
    print(f"OpenCV version: {cv2.__version__}")

    # Specify your ArUco marker size in meters
    # Example: 0.05 is 5cm, 0.10 is 10cm, etc.
    marker_size = 0.06  # Adjust this to your actual marker size in meters

    # Create a detector with the specified marker size
    detector = ArucoDetector(marker_size=marker_size)

    # For better accuracy, calibrate your camera and provide the real parameters:
    # camera_matrix = np.load('camera_matrix.npy')
    # dist_coeffs = np.load('dist_coeffs.npy')
    # detector = ArucoDetector(marker_size=marker_size, camera_matrix=camera_matrix, dist_coeffs=dist_coeffs)

    # Run detection with camera
    # Use 0 for default camera, 1 for external camera
    detector.run_camera(1)