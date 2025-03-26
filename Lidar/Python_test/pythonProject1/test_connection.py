from pyrplidar import PyRPlidar

if __name__ == '__main__':
    lidar = PyRPlidar()
    lidar.connect(port="/dev/ttyUSB0")
    # Linux   : "/dev/ttyUSB0"
    # MacOS   : "/dev/cu.SLAB_USBtoUART"
    # Windows : "COM5"

    info = lidar.get_info()
    print("info :", info)

    health = lidar.get_health()
    print("health :", health)

    samplerate = lidar.get_samplerate()
    print("samplerate :", samplerate)

    scan_modes = lidar.get_scan_modes()
    print("scan modes :")
    for scan_mode in scan_modes:
        print(scan_mode)

    lidar.set_motor_pwm(0)

    lidar.disconnect()