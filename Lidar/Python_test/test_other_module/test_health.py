import serial
from rplidar import RPLidar
from time import sleep

# Open the serial port with increased buffer size
serial_port = '/dev/ttyUSB0'  # Change this to your LIDAR's serial port

lidar = RPLidar(port=serial_port)

# Now you can proceed with your scanning
try:
    lidar.connect()
    print("LIDAR connected successfully.")

    print("Health status:", lidar.get_health())

    sleep(2)

    lidar._set_pwm(500)

    sleep(2)

    for i, scan in enumerate(lidar.iter_scans()):
        print('%d: Got %d measurments' % (i, len(scan)))
        if i > 10:
            break

    lidar._set_pwm(0)


finally:
    lidar._set_pwm(0)
    lidar.stop()
    lidar.disconnect()
