import numpy as np
import cv2
import time
import os


class ArucoDetector:
    def __init__(self, marker_size=0.05, camera_matrix=None, dist_coeffs=None, dictionary_id=cv2.aruco.DICT_4X4_250):
        """
        Initialize the ArUco marker detector

        Parameters:
            marker_size: Physical size of the ArUco marker in meters (default: 0.05m or 5cm)
            camera_matrix: 3x3 camera intrinsic matrix
            dist_coeffs: Camera distortion coefficients
            dictionary_id: ArUco dictionary to use
        """
        # Store the actual marker size in meters
        self.marker_size = marker_size
        print(f"Using marker size: {self.marker_size} meters")

        # Camera calibration - critical for accurate measurements
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs

        # If camera calibration not provided, attempt to load from file or use default
        if self.camera_matrix is None or self.dist_coeffs is None:
            self.load_calibration()

        # Set up ArUco detector for OpenCV 4.11.0
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(dictionary_id)
        self.aruco_params = cv2.aruco.DetectorParameters()

        # Refine parameters for better detection
        self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.aruco_params.cornerRefinementMinAccuracy = 0.05
        self.aruco_params.cornerRefinementWinSize = 5

        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        # For tracking and smoothing
        self.marker_history = {}
        self.smoothing_factor = 0.8  # Higher = more smoothing

    def load_calibration(self):
        """Load camera calibration from files or use default values"""
        calibration_files = [
            ("camera_matrix.npy", "dist_coeffs.npy"),
            ("calibration_matrix.npy", "calibration_dist.npy"),
            ("calib_matrix.npy", "calib_dist.npy")
        ]

        for matrix_file, dist_file in calibration_files:
            if os.path.exists(matrix_file) and os.path.exists(dist_file):
                try:
                    self.camera_matrix = np.load(matrix_file)
                    self.dist_coeffs = np.load(dist_file)
                    print(f"Loaded camera calibration from {matrix_file} and {dist_file}")
                    return
                except Exception as e:
                    print(f"Error loading calibration files: {e}")

        # If no files found or error loading, use default values
        print("Using default camera calibration parameters. For better accuracy, calibrate your camera.")
        # Default camera matrix for a 640x480 camera
        self.camera_matrix = np.array([
            [800, 0, 320],
            [0, 800, 240],
            [0, 0, 1]
        ], dtype=np.float32)
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float32)

    def calibrate_camera(self, images_folder, checkerboard_size=(8, 6), square_size=0.025):
        """
        Calibrate camera using checkerboard images

        Parameters:
            images_folder: Folder containing checkerboard images
            checkerboard_size: Number of internal corners in the checkerboard pattern
            square_size: Size of squares in meters
        """
        print("Starting camera calibration...")

        # Prepare object points
        objp = np.zeros((checkerboard_size[0] * checkerboard_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:checkerboard_size[0], 0:checkerboard_size[1]].T.reshape(-1, 2) * square_size

        # Arrays to store object points and image points
        objpoints = []  # 3D points in real world space
        imgpoints = []  # 2D points in image plane

        # Get list of image files
        image_files = [os.path.join(images_folder, f) for f in os.listdir(images_folder)
                       if f.endswith(".jpg") or f.endswith(".png")]

        if not image_files:
            print("No calibration images found!")
            return False
        print(f"Found {len(image_files)} images for calibration.")

        # Get image dimensions from first image
        img = cv2.imread(image_files[0])
        img_size = (img.shape[1], img.shape[0])
        print(f"Image size: {img_size}")

        # Process each image
        found_count = 0
        for img_file in image_files:
            img = cv2.imread(img_file)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            # Find the checkerboard corners
            ret, corners = cv2.findChessboardCorners(gray, checkerboard_size, None)

            if ret:
                found_count += 1
                # Refine corner positions
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

                objpoints.append(objp)
                imgpoints.append(corners2)

                # Draw and display the corners
                cv2.drawChessboardCorners(img, checkerboard_size, corners2, ret)
                cv2.imshow('Checkerboard Detected', img)
                cv2.waitKey(500)

        cv2.destroyAllWindows()

        if found_count < 5:
            print(f"Not enough checkerboard images found ({found_count}). Need at least 5.")
            return False

        print(f"Found checkerboard patterns in {found_count} images. Calibrating...")

        # Calibrate camera
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
            objpoints, imgpoints, img_size, None, None
        )

        if ret:
            # Save calibration results
            np.save('camera_matrix.npy', mtx)
            np.save('dist_coeffs.npy', dist)

            # Update current instance
            self.camera_matrix = mtx
            self.dist_coeffs = dist

            print("Camera calibration successful!")
            print(f"Camera Matrix:\n{mtx}")
            print(f"Distortion Coefficients:\n{dist.ravel()}")
            return True
        else:
            print("Camera calibration failed!")
            return False

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

        # Apply adaptive thresholding to improve marker detection
        # gray = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2)

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

                # Extract position values
                if tvec.ndim > 1:
                    x = float(tvec[0][0])
                    y = float(tvec[0][1])
                    z = float(tvec[0][2])
                else:
                    x = float(tvec[0])
                    y = float(tvec[1])
                    z = float(tvec[2])

                # Apply temporal smoothing if we have previous data
                if marker_id in self.marker_history:
                    prev = self.marker_history[marker_id]
                    x = self.smoothing_factor * prev['x'] + (1 - self.smoothing_factor) * x
                    y = self.smoothing_factor * prev['y'] + (1 - self.smoothing_factor) * y
                    z = self.smoothing_factor * prev['z'] + (1 - self.smoothing_factor) * z

                # Store current position
                position = {
                    'x': x,
                    'y': y,
                    'z': z,
                    'distance': np.sqrt(x ** 2 + y ** 2 + z ** 2)
                }

                # Update history for smoothing
                self.marker_history[marker_id] = position

                # Store position in results
                marker_positions[marker_id] = position

                # Add text with distance information
                cv2.putText(frame_markers, f"ID:{marker_id} d:{position['distance']:.3f}m",
                            (int(corners[i][0][0][0]), int(corners[i][0][0][1]) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # Add 3D position text
                pos_text = f"X:{position['x']:.3f} Y:{position['y']:.3f} Z:{position['z']:.3f}"
                cv2.putText(frame_markers, pos_text,
                            (int(corners[i][0][0][0]), int(corners[i][0][0][1]) + 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 255), 1)

        return frame_markers, marker_positions

    def run_camera(self, camera_id=0):
        """
        Run a continuous detection loop using the specified camera

        Parameters:
            camera_id: Camera device ID (default: 0)
        """
        cap = cv2.VideoCapture(camera_id)

        if not cap.isOpened():
            print(f"Error: Could not open camera {camera_id}.")
            return

        # Set camera resolution
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        # Get actual resolution
        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(f"Camera {camera_id} opened successfully at resolution {width}x{height}")

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

                # Key handling
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):  # Quit
                    break
                elif key == ord('c'):  # Calibrate
                    cal_folder = input("Enter path to folder with calibration images: ")
                    if os.path.exists(cal_folder):
                        self.calibrate_camera(cal_folder)
                    else:
                        print(f"Folder not found: {cal_folder}")

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
    marker_size = 0.06  # Adjust this to your actual marker size in meters

    # Create a detector with the specified marker size
    detector = ArucoDetector(marker_size=marker_size)

    # Run detection with camera (0 for default, 1 for external)
    detector.run_camera(0)