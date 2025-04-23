#!/usr/bin/env python3

import time
import threading
import logging
from enum import Enum

logger = logging.getLogger(__name__)

# Try to import the lcddriver library, offer installation help if missing
try:
    import lcddriver
    LCD_AVAILABLE = True
except ImportError:
    LCD_AVAILABLE = False
    logger.warning("lcddriver library not found. LCD functionality disabled.")
    logger.info("To install lcddriver, run: pip install lcddriver")

class Team(Enum):
    YELLOW = "YELLOW"
    BLUE = "BLUE"

class LCDInterface:
    """Interface for I2C LCD display to show robot status"""
    
    def __init__(self, i2c_address=0x27, stop_event=None, team=Team.YELLOW):
        """
        Initialize LCD interface
        
        Args:
            i2c_address: I2C address of the LCD (default: 0x27)
            stop_event: Threading event to signal when to stop the LCD thread
            team: Team color (YELLOW or BLUE)
        """
        self.stop_event = stop_event if stop_event else threading.Event()
        self.lcd = None
        self.team = team
        self.current_state = "UNKNOWN"
        self.target_location = "UNKNOWN"
        self.current_task = "NONE"
        self.update_lock = threading.Lock()
        self.lcd_thread = None
        
        if LCD_AVAILABLE:
            try:
                self.lcd = lcddriver.lcd(i2c_address)
                self.lcd.lcd_clear()
                logger.info(f"LCD initialized at address 0x{i2c_address:02x}")
                self._display_intro()
            except Exception as e:
                logger.error(f"Error initializing LCD: {e}")
                LCD_AVAILABLE = False
                self.lcd = None
    
    def _display_intro(self):
        """Display initial splash screen"""
        if not self.lcd:
            return
            
        self.lcd.lcd_clear()
        self.lcd.lcd_display_string("OverTheRainBot", 1)
        self.lcd.lcd_display_string(f"Team: {self.team.value}", 2)
        time.sleep(2)
    
    def update_state(self, state):
        """Update the current robot state"""
        with self.update_lock:
            self.current_state = state
    
    def update_target(self, target):
        """Update the target location"""
        with self.update_lock:
            self.target_location = target
    
    def update_task(self, task):
        """Update the current task"""
        with self.update_lock:
            self.current_task = task
    
    def set_team(self, team):
        """Set the team color"""
        if team in [Team.YELLOW, Team.BLUE]:
            with self.update_lock:
                self.team = team
        else:
            logger.error(f"Invalid team: {team}")
    
    def _update_display(self):
        """Update the LCD display"""
        if not self.lcd:
            return
            
        with self.update_lock:
            # First line: Team and State
            team_str = "Y" if self.team == Team.YELLOW else "B"
            line1 = f"T:{team_str} S:{self.current_state[:14]}"
            
            # Second line: Target location or current task
            if self.current_state == "EXECUTING_TASK" and self.current_task != "NONE":
                line2 = f"Task:{self.current_task[:15]}"
            else:
                line2 = f"To:{self.target_location[:17]}"
        
        try:
            self.lcd.lcd_clear()
            self.lcd.lcd_display_string(line1[:20], 1)  # Truncate to 20 chars if needed
            self.lcd.lcd_display_string(line2[:20], 2)  # Truncate to 20 chars if needed
        except Exception as e:
            logger.error(f"Error updating LCD: {e}")
    
    def start(self):
        """Start the LCD update thread"""
        if not LCD_AVAILABLE:
            logger.warning("LCD not available, not starting LCD thread")
            return
            
        if self.lcd_thread and self.lcd_thread.is_alive():
            logger.warning("LCD thread already running")
            return
            
        self.stop_event.clear()
        self.lcd_thread = threading.Thread(target=self._lcd_loop)
        self.lcd_thread.daemon = True
        self.lcd_thread.start()
        logger.info("LCD interface thread started")
    
    def stop(self):
        """Stop the LCD update thread"""
        if self.stop_event:
            self.stop_event.set()
        if self.lcd_thread and self.lcd_thread.is_alive():
            self.lcd_thread.join(timeout=1)
        
        # Display goodbye message
        if self.lcd:
            try:
                self.lcd.lcd_clear()
                self.lcd.lcd_display_string("OverTheRainBot", 1)
                self.lcd.lcd_display_string("Shutting down...", 2)
                time.sleep(1)
                self.lcd.lcd_clear()
            except Exception:
                pass
    
    def _lcd_loop(self):
        """Main LCD update loop"""
        update_interval = 0.5  # seconds between updates
        
        while not self.stop_event.is_set():
            try:
                self._update_display()
                time.sleep(update_interval)
            except Exception as e:
                logger.error(f"Error in LCD update loop: {e}")
                time.sleep(1)  # Wait a bit longer on error
                
        logger.info("LCD interface thread stopped")

# For testing
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    lcd = LCDInterface()
    lcd.start()
    
    # Test different states and information
    lcd.update_state("IDLE")
    lcd.update_target("Starting Point")
    time.sleep(2)
    
    lcd.update_state("NAVIGATING")
    lcd.update_target("Checkpoint 1")
    time.sleep(2)
    
    lcd.update_state("EXECUTING_TASK")
    lcd.update_task("Grabbing object")
    time.sleep(2)
    
    lcd.update_state("AVOIDING")
    lcd.update_target("Destination")
    time.sleep(2)
    
    lcd.stop()
