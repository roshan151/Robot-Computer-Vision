"""
Improved Drivetrain Control with Proper Encoder Synchronization
==============================================================

This improved version addresses the issues in the original drivetrain.py:
1. Proper quadrature encoder reading with direction detection
2. Interrupt-based encoder counting for accuracy
3. PID control for better motor synchronization
4. Better code structure and error handling
5. Real-time encoder monitoring

Author: AI Assistant (Improved version)
"""

import time
import threading
import RPi.GPIO as GPIO
from collections import deque
import math


class QuadratureEncoder:
    """Handle quadrature encoder with proper A/B channel reading"""
    
    def __init__(self, pin_a, pin_b, name="Encoder"):
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.name = name
        
        # Encoder state
        self.count = 0
        self.direction = 1  # 1 for forward, -1 for reverse
        self.last_a = 0
        self.last_b = 0
        
        # Setup GPIO
        GPIO.setup(self.pin_a, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.pin_b, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
        # Initialize previous states
        self.last_a = GPIO.input(self.pin_a)
        self.last_b = GPIO.input(self.pin_b)
        
        # Add interrupt callbacks for both channels
        GPIO.add_event_detect(self.pin_a, GPIO.BOTH, callback=self._encoder_callback, bouncetime=1)
        GPIO.add_event_detect(self.pin_b, GPIO.BOTH, callback=self._encoder_callback, bouncetime=1)
        
        # Thread lock for thread-safe operations
        self.lock = threading.Lock()
    
    def _encoder_callback(self, channel):
        """Interrupt callback for encoder changes"""
        with self.lock:
            # Read current states
            a = GPIO.input(self.pin_a)
            b = GPIO.input(self.pin_b)
            
            # Determine direction and count based on quadrature encoding
            if channel == self.pin_a:
                if a != self.last_a:
                    if a == b:
                        self.direction = 1  # Forward
                        self.count += 1
                    else:
                        self.direction = -1  # Reverse
                        self.count -= 1
                    self.last_a = a
            
            elif channel == self.pin_b:
                if b != self.last_b:
                    if a != b:
                        self.direction = 1  # Forward
                        self.count += 1
                    else:
                        self.direction = -1  # Reverse
                        self.count -= 1
                    self.last_b = b
    
    def get_count(self):
        """Get current encoder count (thread-safe)"""
        with self.lock:
            return self.count
    
    def reset_count(self):
        """Reset encoder count to zero"""
        with self.lock:
            self.count = 0
    
    def get_direction(self):
        """Get current direction"""
        with self.lock:
            return self.direction


class PIDController:
    """PID Controller for motor synchronization"""
    
    def __init__(self, kp=1.0, ki=0.1, kd=0.05, setpoint=0.0):
        self.kp = kp  # Proportional gain
        self.ki = ki  # Integral gain
        self.kd = kd  # Derivative gain
        self.setpoint = setpoint
        
        # PID state
        self.previous_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
        
        # Error history for debugging
        self.error_history = deque(maxlen=100)
    
    def update(self, current_value):
        """Update PID controller with current value"""
        current_time = time.time()
        dt = current_time - self.last_time
        
        if dt <= 0.0:
            return 0.0
        
        # Calculate error
        error = self.setpoint - current_value
        
        # Proportional term
        proportional = self.kp * error
        
        # Integral term (with windup protection)
        self.integral += error * dt
        self.integral = max(-100, min(100, self.integral))  # Limit integral windup
        integral = self.ki * self.integral
        
        # Derivative term
        derivative = self.kd * (error - self.previous_error) / dt
        
        # Calculate output
        output = proportional + integral + derivative
        
        # Store for next iteration
        self.previous_error = error
        self.last_time = current_time
        self.error_history.append(error)
        
        return output
    
    def reset(self):
        """Reset PID controller state"""
        self.previous_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
        self.error_history.clear()


class Drivetrain:
    """Improved drivetrain with proper encoder synchronization"""
    
    def __init__(self):
        # GPIO pin numbers for motors
        self.M1_ENA, self.M1_IN1, self.M1_IN2 = 25, 23, 24
        self.M2_ENA, self.M2_IN1, self.M2_IN2 = 22, 27, 17
        
        # GPIO pin numbers for encoders
        self.EN1_A, self.EN1_B = 20, 21
        self.EN2_A, self.EN2_B = 19, 26
        
        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        self._setup_motors()
        self._setup_encoders()
        self._setup_pid_controllers()
        
        # Control parameters
        self.base_speed = 70
        self.max_speed = 90
        self.min_speed = 40
        self.control_frequency = 50  # Hz
        
        # State tracking
        self.is_moving = False
        self.movement_thread = None
        
        print("Drivetrain initialized successfully")
    
    def _setup_motors(self):
        """Setup motor GPIO pins and PWM"""
        # Motor 1
        GPIO.setup(self.M1_ENA, GPIO.OUT)
        GPIO.setup(self.M1_IN1, GPIO.OUT)
        GPIO.setup(self.M1_IN2, GPIO.OUT)
        self.pwm_1 = GPIO.PWM(self.M1_ENA, 1000)  # 1kHz PWM frequency
        self.pwm_1.start(0)
        
        # Motor 2
        GPIO.setup(self.M2_ENA, GPIO.OUT)
        GPIO.setup(self.M2_IN1, GPIO.OUT)
        GPIO.setup(self.M2_IN2, GPIO.OUT)
        self.pwm_2 = GPIO.PWM(self.M2_ENA, 1000)  # 1kHz PWM frequency
        self.pwm_2.start(0)
    
    def _setup_encoders(self):
        """Setup quadrature encoders"""
        self.encoder_1 = QuadratureEncoder(self.EN1_A, self.EN1_B, "Motor1")
        self.encoder_2 = QuadratureEncoder(self.EN2_A, self.EN2_B, "Motor2")
    
    def _setup_pid_controllers(self):
        """Setup PID controllers for synchronization"""
        # PID for keeping motors in sync (error = difference in encoder counts)
        self.sync_pid = PIDController(kp=0.8, ki=0.2, kd=0.1, setpoint=0.0)
        
        # Separate PIDs for each motor speed control
        self.motor1_pid = PIDController(kp=1.0, ki=0.1, kd=0.05)
        self.motor2_pid = PIDController(kp=1.0, ki=0.1, kd=0.05)
    
    def _set_motor_direction(self, motor_num, direction):
        """Set motor direction (1=forward, -1=reverse, 0=stop)"""
        if motor_num == 1:
            if direction == 1:  # Forward
                GPIO.output(self.M1_IN1, GPIO.HIGH)
                GPIO.output(self.M1_IN2, GPIO.LOW)
            elif direction == -1:  # Reverse
                GPIO.output(self.M1_IN1, GPIO.LOW)
                GPIO.output(self.M1_IN2, GPIO.HIGH)
            else:  # Stop
                GPIO.output(self.M1_IN1, GPIO.LOW)
                GPIO.output(self.M1_IN2, GPIO.LOW)
                
        elif motor_num == 2:
            if direction == 1:  # Forward
                GPIO.output(self.M2_IN1, GPIO.LOW)
                GPIO.output(self.M2_IN2, GPIO.HIGH)
            elif direction == -1:  # Reverse
                GPIO.output(self.M2_IN1, GPIO.HIGH)
                GPIO.output(self.M2_IN2, GPIO.LOW)
            else:  # Stop
                GPIO.output(self.M2_IN1, GPIO.LOW)
                GPIO.output(self.M2_IN2, GPIO.LOW)
    
    def _controlled_movement(self, direction, duration, base_speed):
        """Execute controlled movement with encoder synchronization"""
        # Reset encoders and PIDs
        self.encoder_1.reset_count()
        self.encoder_2.reset_count()
        self.sync_pid.reset()
        
        # Set motor directions
        self._set_motor_direction(1, direction)
        self._set_motor_direction(2, direction)
        
        # Control loop
        start_time = time.time()
        control_period = 1.0 / self.control_frequency
        
        m1_speed = base_speed
        m2_speed = base_speed
        
        while (time.time() - start_time) < duration and self.is_moving:
            loop_start = time.time()
            
            # Get encoder counts
            count_1 = self.encoder_1.get_count()
            count_2 = self.encoder_2.get_count()
            
            # Calculate synchronization error
            sync_error = count_1 - count_2
            
            # Get PID correction
            correction = self.sync_pid.update(sync_error)
            
            # Apply correction to motor speeds
            m1_speed = base_speed - (correction / 2)
            m2_speed = base_speed + (correction / 2)
            
            # Limit speeds to safe range
            m1_speed = max(self.min_speed, min(self.max_speed, m1_speed))
            m2_speed = max(self.min_speed, min(self.max_speed, m2_speed))
            
            # Set motor speeds
            self.pwm_1.ChangeDutyCycle(abs(m1_speed))
            self.pwm_2.ChangeDutyCycle(abs(m2_speed))
            
            # Debug output (uncomment for debugging)
            # if int(time.time() * 4) % 4 == 0:  # Print every 0.25 seconds
            #     print(f"Counts: M1={count_1}, M2={count_2}, Error={sync_error:.1f}, "
            #           f"Speeds: M1={m1_speed:.1f}, M2={m2_speed:.1f}")
            
            # Maintain control frequency
            elapsed = time.time() - loop_start
            if elapsed < control_period:
                time.sleep(control_period - elapsed)
        
        # Stop motors
        self.stop()
    
    def straight(self, duration=3, speed=70):
        """Move straight forward with encoder synchronization"""
        print(f"Moving straight for {duration}s at speed {speed}")
        self.is_moving = True
        self._controlled_movement(1, duration, speed)
    
    def reverse(self, duration=3, speed=70):
        """Move reverse with encoder synchronization"""
        print(f"Moving reverse for {duration}s at speed {speed}")
        self.is_moving = True
        self._controlled_movement(-1, duration, speed)
    
    def right(self, angle=90, speed=50):
        """Turn right by specified angle"""
        print(f"Turning right {angle} degrees at speed {speed}")
        
        # Calculate required encoder counts for the turn
        # This needs calibration based on your robot's wheel base and encoder resolution
        counts_per_degree = 80 / 9  # From your original code - calibrate this!
        target_counts = int(counts_per_degree * angle)
        
        # Reset encoders
        self.encoder_1.reset_count()
        self.encoder_2.reset_count()
        
        # Set motor directions for right turn (left wheel forward, right wheel reverse)
        self._set_motor_direction(1, -1)  # Motor 1 reverse
        self._set_motor_direction(2, -1)  # Motor 2 reverse (both reverse for right turn)
        
        # Control loop
        while max(abs(self.encoder_1.get_count()), abs(self.encoder_2.get_count())) < target_counts:
            self.pwm_1.ChangeDutyCycle(speed)
            self.pwm_2.ChangeDutyCycle(speed)
            time.sleep(0.01)  # Small delay
        
        self.stop()
    
    def left(self, angle=90, speed=50):
        """Turn left by specified angle"""
        print(f"Turning left {angle} degrees at speed {speed}")
        
        # Calculate required encoder counts
        counts_per_degree = 80 / 9  # Calibrate this!
        target_counts = int(counts_per_degree * angle)
        
        # Reset encoders
        self.encoder_1.reset_count()
        self.encoder_2.reset_count()
        
        # Set motor directions for left turn (both forward)
        self._set_motor_direction(1, 1)  # Motor 1 forward
        self._set_motor_direction(2, 1)  # Motor 2 forward
        
        # Control loop
        while max(abs(self.encoder_1.get_count()), abs(self.encoder_2.get_count())) < target_counts:
            self.pwm_1.ChangeDutyCycle(speed)
            self.pwm_2.ChangeDutyCycle(speed)
            time.sleep(0.01)
        
        self.stop()
    
    def stop(self):
        """Stop all motors"""
        self.is_moving = False
        self.pwm_1.ChangeDutyCycle(0)
        self.pwm_2.ChangeDutyCycle(0)
        self._set_motor_direction(1, 0)
        self._set_motor_direction(2, 0)
        print("Motors stopped")
    
    def get_encoder_status(self):
        """Get current encoder readings for debugging"""
        return {
            "motor1_count": self.encoder_1.get_count(),
            "motor2_count": self.encoder_2.get_count(),
            "motor1_direction": self.encoder_1.get_direction(),
            "motor2_direction": self.encoder_2.get_direction(),
            "sync_error": self.encoder_1.get_count() - self.encoder_2.get_count()
        }
    
    def calibrate_turn(self, angle=90, speed=50):
        """Calibrate turn parameters by measuring actual encoder counts"""
        print(f"Calibrating turn for {angle} degrees...")
        
        self.encoder_1.reset_count()
        self.encoder_2.reset_count()
        
        # Perform the turn
        self.right(angle, speed)
        
        # Measure encoder counts
        final_count_1 = abs(self.encoder_1.get_count())
        final_count_2 = abs(self.encoder_2.get_count())
        avg_count = (final_count_1 + final_count_2) / 2
        
        counts_per_degree = avg_count / angle
        
        print(f"Turn calibration results:")
        print(f"  Motor 1 counts: {final_count_1}")
        print(f"  Motor 2 counts: {final_count_2}")
        print(f"  Average counts: {avg_count}")
        print(f"  Counts per degree: {counts_per_degree:.2f}")
        print(f"  Update your code with: counts_per_degree = {counts_per_degree:.2f}")
        
        return counts_per_degree
    
    def cleanup(self):
        """Clean up GPIO and stop all operations"""
        self.stop()
        GPIO.cleanup()
        print("GPIO cleanup completed")


# Example usage and testing
def main():
    """Test the drivetrain"""
    try:
        # Initialize drivetrain
        robot = Drivetrain()
        
        print("Testing drivetrain...")
        print("=" * 40)
        
        # Test movements
        print("1. Testing straight movement...")
        robot.straight(duration=2, speed=60)
        status = robot.get_encoder_status()
        print(f"   Encoder status: {status}")
        
        time.sleep(1)
        
        print("2. Testing reverse movement...")
        robot.reverse(duration=2, speed=60)
        status = robot.get_encoder_status()
        print(f"   Encoder status: {status}")
        
        time.sleep(1)
        
        print("3. Testing right turn...")
        robot.right(angle=90, speed=50)
        
        time.sleep(1)
        
        print("4. Testing left turn...")
        robot.left(angle=90, speed=50)
        
        print("Testing completed successfully!")
        
        # Optional: Calibrate turns
        # print("5. Calibrating turn...")
        # robot.calibrate_turn(90, 50)
        
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    except Exception as e:
        print(f"Error during testing: {e}")
    finally:
        robot.cleanup()


if __name__ == "__main__":
    main()
