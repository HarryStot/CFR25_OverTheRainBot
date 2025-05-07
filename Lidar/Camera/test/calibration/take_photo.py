import cv2
from time import sleep


# Take 10 photos with a 2-second interval
def take_photos(num_photos=10, interval=2):
    # Initialize video capture (0 = default webcam)
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    for i in range(num_photos):
        ret, frame = cap.read()
        if not ret:
            print(f"Error: Could not read frame {i}.")
            break

        # Save the frame as an image file
        filename = f"photo_{i + 1}.jpg"
        cv2.imwrite(filename, frame)
        print(f"Saved {filename}")

        # Wait for the specified interval before taking the next photo
        sleep(interval)

    # Cleanup
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    take_photos(num_photos=30, interval=2)
