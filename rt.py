#!/usr/bin/env python3
"""
EV Control System for 4-wheel vehicle with rear-wheel drive
- 2 NEMA 17 stepper motors (rear wheels)
- 2 omni wheels (front wheels, unpowered)
- TMC2209 stepper motor drivers
- Belt drive with gear ratio 13:10 (motor:wheel)
- Wheel diameter: 5cm
- Back board   - Left motor
- Front board  - Right motor
"""

import RPi.GPIO as GPIO
import time
import math
import threading
from TMC_2209.TMC_2209_StepperDriver import *

class EV:
    def __init__(self, wheel_diameter=6.1):

        # Motor specifications
        self.STEPS_PER_REV = 200        # NEMA 17 standard
        self.MICROSTEPS = 8             # TMC2209 microstepping
        self.ACTUAL_STEPS_PER_REV = self.STEPS_PER_REV * self.MICROSTEPS  # 1600 steps
        
        # Mechanical specifications
        # adjustment
        # short 8cm/70cm, new_diameter = old_diameter / (1+8/70)
        # long  8cm/70cm, new diameter = old_diameter * (1+8/70) 
        self.WHEEL_DIAMETER_CM = wheel_diameter    # Wheel diameter in cm
        self.WHEEL_CIRCUMFERENCE_CM = self.WHEEL_DIAMETER_CM * math.pi
        
        # Gear ratio: motor gear teeth / wheel gear teeth
        # If motor has 13 teeth and wheel has 10 teeth:
        # When motor turns 1 revolution, wheel turns 13/10 = 1.3 revolutions
        # So wheel moves MORE than motor rotation
        # Therefore: wheel_distance = motor_steps / gear_ratio
        self.MOTOR_GEAR_TEETH = 5.0
        self.WHEEL_GEAR_TEETH = 10.0
        self.GEAR_RATIO = self.MOTOR_GEAR_TEETH / self.WHEEL_GEAR_TEETH  # 1.3
        
        # Calculate steps per cm of wheel movement
        # Since wheel moves MORE than motor due to gear ratio, we need FEWER motor steps per cm
        self.STEPS_PER_CM = self.ACTUAL_STEPS_PER_REV / (self.WHEEL_CIRCUMFERENCE_CM * self.GEAR_RATIO)
        
        print(f"Wheel circumference: {self.WHEEL_CIRCUMFERENCE_CM:.2f} cm")
        print(f"Gear ratio: {self.GEAR_RATIO:.2f} (wheel turns {self.GEAR_RATIO}x more than motor)")
        print(f"Steps per cm: {self.STEPS_PER_CM:.2f}")
        print(f"Expected steps for 50cm: {50 * self.STEPS_PER_CM:.0f}")
        
        
        # GPIO pin definitions (adjust these to your wiring)
        self.LEFT_DIR_PIN = 23      # Left motor direction
        self.LEFT_STEP_PIN = 24     # Left motor step
        self.LEFT_ENABLE_PIN = 25   # Left motor enable
        
        self.RIGHT_DIR_PIN = 6      # Right motor direction  
        self.RIGHT_STEP_PIN = 13    # Right motor step
        self.RIGHT_ENABLE_PIN = 26  # Right motor enable
        
        # Initialize GPIO and TMC drivers
        self.setup_gpio()
        self.setup_tmc_drivers()

        
    def setup_gpio(self):
        """Initialize GPIO pins for motor control"""
        GPIO.cleanup()
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

      
        return
    
        # Setup motor pins as outputs
        pins = [self.LEFT_DIR_PIN, self.LEFT_STEP_PIN, self.LEFT_ENABLE_PIN,
                self.RIGHT_DIR_PIN, self.RIGHT_STEP_PIN, self.RIGHT_ENABLE_PIN]
        
        for pin in pins:
            GPIO.setup(pin, GPIO.OUT)
        
        # Enable motors (LOW = enabled for TMC2209)
        GPIO.output(self.LEFT_ENABLE_PIN, GPIO.LOW)
        GPIO.output(self.RIGHT_ENABLE_PIN, GPIO.LOW)
        
        print("EV GPIO initialized and motors enabled")
        
        
    def setup_tmc_drivers(self):
        """Initialize TMC2209 drivers with speed control"""
        print("Configuring TMC2209 drivers...")
        
        # Create TMC driver instances
        # driver_address = 0 for left motor (MS1=GND, MS2=GND)
        # driver_address = 1 for right motor (MS1=VCC, MS2=GND)

        self.tmc_left = TMC_2209(pin_en=self.LEFT_ENABLE_PIN, pin_step=self.LEFT_STEP_PIN, 
                                pin_dir=self.LEFT_DIR_PIN, driver_address=0)
        print("Left TMC2209 initialized")
        self.tmc_right = TMC_2209(pin_en=self.RIGHT_ENABLE_PIN, pin_step=self.RIGHT_STEP_PIN, 
                                 pin_dir=self.RIGHT_DIR_PIN, driver_address=1)
        print("Right TMC2209 initialized")
        # Configure both drivers
        for tmc in [self.tmc_left, self.tmc_right]:
            print("Setting up TMC")
            # Set motor current (adjust based on your motor specs)
            tmc.set_current(400)  # 1.2A RMS for NEMA 17
            print("current set")
            # Enable microstep interpolation for smoother motion
            tmc.set_interpolation(True)
            
            # Use stealthChop for quiet operation
            tmc.set_spreadcycle(False)
            
            # Set microstepping resolution
            tmc.set_microstepping_resolution(self.MICROSTEPS)
            
            # Use external sense resistor
            tmc.set_internal_rsense(False)
            
            # Set acceleration (steps/sec²)
            tmc.set_acceleration(1000)
        
        # Set direction registers
        self.tmc_left.set_direction_reg(False)   # Normal direction
        self.tmc_right.set_direction_reg(False)   # Opposite for right motor
        
        print("TMC2209 drivers configured")
        
 
    def move(self, distance, seconds):
        """
        Move the EV forward or backward using TMC2209 speed control with threading
        
        Args:
            distance (float): Distance to move in cm (positive = forward, negative = backward)
            seconds (float): Time to complete the movement in seconds
        """
        print(f"Moving {distance}cm in {seconds} seconds using threaded TMC speed control")
        print(f"Start moving time {time.time()}")
        
        # Calculate movement parameters
        speed_cm_s = abs(distance) / seconds
        steps_needed = int(abs(distance) * self.STEPS_PER_CM)
        steps_per_second = steps_needed / seconds
        
        print(f"Speed: {speed_cm_s:.2f} cm/s")
        print(f"Steps needed: {steps_needed}")
        print(f"Motor speed: {steps_per_second:.2f} steps/second")
        
        # Set motor speeds
        self.tmc_left.set_max_speed(int(steps_per_second))
        self.tmc_right.set_max_speed(int(steps_per_second))
        
        # Set direction
        if distance >= 0:
            # Forward - both motors same direction
            self.tmc_left.set_direction_reg(True)
            self.tmc_right.set_direction_reg(True)

        else:
            # Backward - reverse both motors
            self.tmc_left.set_direction_reg(False)
            self.tmc_right.set_direction_reg(False)

        time.sleep(0.05)  # Short delay to ensure direction is set before starting motion       

        # Calculate target steps
        target_steps = steps_needed if distance >= 0 else -steps_needed
        
        print("Starting threaded coordinated movement...")
        start_time = time.time()
        
        # Enable motors
        self.tmc_left.set_motor_enabled(True)
        self.tmc_right.set_motor_enabled(True)
        
        # Define motor movement functions
        def move_left_motor():
            try:

                self.tmc_left.run_to_position_steps(target_steps, steps_per_second)
                print("Left  motor movement completed")
            except Exception as e:
                print(f"Left motor error: {e}")
        
        def move_right_motor():
            try:
                print("Right motor target steps:",target_steps, steps_per_second)
                self.tmc_right.run_to_position_steps(target_steps, steps_per_second)
                print("Right motor movement completed")
            except Exception as e:
                print(f"Right motor error: {e}")
        
        # Create and start threads for both motors
        left_thread = threading.Thread(target=move_left_motor)
        right_thread = threading.Thread(target=move_right_motor)
        

        # Start both motors simultaneously
        left_thread.start()
        right_thread.start()

 


        # Wait for both motors to complete
        left_thread.join()
        right_thread.join()



        # Disable motors
        self.tmc_left.set_motor_enabled(False)
        self.tmc_right.set_motor_enabled(False)
        
        elapsed_time = time.time() - start_time
        actual_speed = abs(distance) / elapsed_time
        
        print(f"Movement completed in {elapsed_time:.2f} seconds")
        print(f"Actual speed: {actual_speed:.2f} cm/s")
        
        
    def cleanup(self):
        """Clean up GPIO resources"""
        # Stop motors
        try:
            self.tmc_left.stop()
            self.tmc_right.stop()
        except:
            pass
            
        # Disable motors
        #GPIO.output(self.LEFT_ENABLE_PIN, GPIO.HIGH)
        #GPIO.output(self.RIGHT_ENABLE_PIN, GPIO.HIGH)
        GPIO.cleanup()
        print("EV cleaned up")

# Functions outside the EV class
def run_program(filename="commands.txt"):
    """
    Reads commands from a text file and executes them on the given EV instance.
    Supported commands (case-insensitive, one per line):
    - forward x t   (move forward x cm in t seconds)
    - back x t      (move backward x cm in t seconds)
    - left x        (turn CCW x degrees)
    - right x       (turn CW x degrees)
    Lines starting with '#' are treated as comments and ignored.
    Unrecognized commands are ignored.
    """
    try:
        program_start_time = time.time()
        print(f"program start time '{program_start_time}' ")
        ev = None
        diameter=0
        time_compensation = 2
        with open(filename, 'r') as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                parts = line.lower().split()
                if len(parts) == 0:
                    continue
                cmd = parts[0]
                if cmd == 'ev_diameter' and len(parts) == 2:
                    try:
                        diameter = float(parts[1])
                    except Exception as e:
                        print(f"Invalid ev_diameter command: {line} ({e})")
                elif cmd == "ev_init":

                    ev = EV(wheel_diameter=diameter if diameter>0 else 6.3643)
                    if len(parts)==2:
                        ev.REVERSE = True
                    if ev:
                        ev.cleanup()
                    print(f"Initialized EV with wheel diameter: {ev.WHEEL_DIAMETER_CM} cm")

                elif cmd == 'forward' and len(parts) == 3:
                    try:
                        if ev is None:
                            ev = EV(wheel_diameter=diameter if diameter>0 else 6.3643)
                            print(f"Initialized EV with wheel diameter: {ev.WHEEL_DIAMETER_CM} cm")
                        dist = float(parts[1])
                        t = float(parts[2])
                        print(f"Command: FORWARD {dist}cm in {t}s")
                        ev.move(dist, t-time_compensation)
                    except Exception as e:
                        print(f"Invalid forward command: {line} ({e})")
                elif cmd == 'left' and len(parts) == 2:
                    try:
                        deg = float(parts[1])
                        print(f"Command: LEFT {deg} degrees (CCW)")
                        ev.turn(-deg)
                    except Exception as e:
                        print(f"Invalid left command: {line} ({e})")
                elif cmd == 'right' and len(parts) == 2:
                    try:
                        deg = float(parts[1])
                        print(f"Command: RIGHT {deg} degrees (CW)")
                        ev.turn(deg)
                    except Exception as e:
                        print(f"Invalid right command: {line} ({e})")
                else:
                    print(f"Ignoring unrecognized command: {line}")
        program_end_time = time.time()
        print(f"Program completed in {program_end_time - program_start_time:.2f} seconds")
        if ev:
            ev.cleanup()
    except FileNotFoundError:
        print(f"Command file '{filename}' not found.")
    except Exception as e:
        print(f"Error reading commands: {e}")

# Example usage and test functions
def test():
    """Test basic forward/backward movement"""
    ev = EV(wheel_diameter=6.3643)  # Uses default wheel diameter
    
    try:
        print("=== Testing Basic Movement ===")
        
        # Move forward 20cm in 2 seconds (10 cm/s)
        ev.move(200, 10)
        time.sleep(1)
        
    except KeyboardInterrupt:
        print("Test interrupted")
    finally:
        ev.cleanup()


if __name__ == "__main__":
    try:
        test()
    except KeyboardInterrupt:
        print("\nProgram interrupted")
    except Exception as e:
        print(f"Error: {e}")
