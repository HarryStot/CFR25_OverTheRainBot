import serial, time

if __name__ == '__main__':

    print('Running. Press CTRL-C to exit.')
    with serial.Serial("/dev/ttyACM0", 115200, timeout=1) as arduino:
        time.sleep(0.1)  # wait for serial to open
        if arduino.is_open:
            print("{} connected!".format(arduino.port))
            try:
                while True:
                    cmd = input("Enter command: ")
                    print(cmd)
                    arduino.write(cmd.encode())
                    # time.sleep(0.1) #wait for arduino to answer
                    while arduino.in_waiting == 0: pass
                    if arduino.in_waiting > 0:
                        answer = arduino.readline()
                        print(answer)
                        arduino.reset_input_buffer()  # remove data after reading
            except KeyboardInterrupt:
                print("KeyboardInterrupt has been caught.")