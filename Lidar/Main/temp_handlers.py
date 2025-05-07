def handle_team_selection_state(self):
    """Handle the TEAM_SELECTION state - determine team based on switch position"""
    # Check team selection switch
    if GPIO.input(self.team_select_pin) == GPIO.HIGH:
        self.is_blue_team = True
        logger.info("Blue team selected")
    else:
        self.is_blue_team = False
        logger.info("Yellow team selected")

    # Load mission locations based on team
    self.load_team_missions()
    
    # Move to waiting for start
    self.set_state(RobotState.WAITING_FOR_START)

def handle_waiting_for_start_state(self):
    """Handle the WAITING_FOR_START state - wait for pull switch activation"""
    # Check if pull switch has been activated
    if GPIO.input(self.pull_switch_pin) == GPIO.LOW:
        logger.info("Pull switch activated! Starting mission...")
        self.mission_start_time = time.time()
        
        # Start the mission with the first location
        self.current_location_index = 0
        self.current_task_index = -1
        
        # Move to navigating state to go to first location
        self.set_state(RobotState.NAVIGATING)
        self.navigation_start_time = time.time()
    else:
        # Still waiting for pull switch activation
        time.sleep(0.1)

def load_team_missions(self):
    """Load mission locations based on selected team"""
    self.clear_locations()
    
    if self.is_blue_team:
        # Blue team mission waypoints
        self.add_location(Location("BlueStart", 30, 30, 0))
        self.add_location(Location("BlueActionPoint1", 50, 70, 90, [
            Task("GrabBlueItem", "GRAB", {"S": 1}, 3)
        ]))
        self.add_location(Location("BlueActionPoint2", 90, 80, 180, [
            Task("DropBlueItem", "DROP", {"S": 1}, 2)
        ]))
        # Add more blue team locations as needed
        self.add_location(Location("BlueEndZone", 180, 30, 270))
    else:
        # Yellow team mission waypoints
        self.add_location(Location("YellowStart", 270, 30, 180))
        self.add_location(Location("YellowActionPoint1", 250, 70, 90, [
            Task("GrabYellowItem", "GRAB", {"S": 2}, 3)
        ]))
        self.add_location(Location("YellowActionPoint2", 210, 80, 0, [
            Task("DropYellowItem", "DROP", {"S": 2}, 2)
        ]))
        # Add more yellow team locations as needed
        self.add_location(Location("YellowEndZone", 120, 30, 270))
        
    logger.info(f"Loaded {len(self.locations)} locations for {'blue' if self.is_blue_team else 'yellow'} team")

def handle_returning_to_end_state(self):
    """Handle the RETURNING_TO_END state - go to end zone before time runs out"""
    # Get the end zone location (last location in the list)
    end_location = self.locations[-1]
    
    # Get current position
    current_x, current_y, current_z = position_manager.get_position()
    distance = ((current_x - end_location.x) ** 2 +
               (current_y - end_location.y) ** 2) ** 0.5
    
    # Check if we've reached the end zone
    if distance <= self.position_tolerance:
        logger.info("Reached end zone! Mission completed.")
        self.send_movement_command("S")  # Stop the robot
        self.set_state(RobotState.COMPLETED)
        return
        
    # Continue navigating to end zone
    self.send_movement_command(f"GX{end_location.x:.2f}Y{end_location.y:.2f}Z{end_location.orientation:.2f}")
    logger.info(f"Returning to end zone, distance: {distance:.2f}")
    
    # Check if we're about to run out of time
    elapsed_time = time.time() - self.mission_start_time
    remaining_time = self.mission_duration - elapsed_time
    
    if remaining_time < 2:
        # Almost out of time, emergency stop
        logger.warning("Mission time almost up! Performing emergency stop.")
        self.send_movement_command("S")
        self.set_state(RobotState.COMPLETED)

def handle_completed_state(self):
    """Handle the COMPLETED state - mission is done"""
    # Just stop the robot and wait
    self.send_movement_command("S")
    time.sleep(0.5)
    # Idle until shutdown
    pass
