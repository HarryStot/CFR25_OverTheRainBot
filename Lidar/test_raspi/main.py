from robot_controller import RobotController

if __name__ == "__main__":
    print("Starting robot control system...")
    try:
        robot = RobotController()
        robot.start()
    except Exception as e:
        print(f"Error: {e}")
        # Emergency stop in case of error
        try:
            robot.stop()
        except:
            pass