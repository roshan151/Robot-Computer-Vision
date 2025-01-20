import time
import RPi.GPIO as GPIO

class movement:

    # GPIO pin numbers for motors
    Ena_M1, M1_In1, M1_In2 = 25, 23, 24
    Ena_M2, M2_In1, M2_In2 = 22, 27, 17
    
    # GPIO pin numbers for encoders
    En_1_A, En_1_B = 20, 21
    En_2_A, En_2_B = 19, 26

    # A discrepancy between encoder counts beyond this threshold will trigger a pwm correction.
    correction_threshold = 50
    
    def __init__(self):
        # Setup GPIO pins for both motors
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # Motor 1
        GPIO.setup(self.Ena_M1, GPIO.OUT)
        GPIO.setup(self.M1_In1, GPIO.OUT)
        GPIO.setup(self.M1_In2, GPIO.OUT)
        self.pwm_1 = GPIO.PWM(self.Ena_M1, 100)
        self.pwm_1.start(0)

        # Motor 2
        GPIO.setup(self.Ena_M2, GPIO.OUT)
        GPIO.setup(self.M2_In1, GPIO.OUT)
        GPIO.setup(self.M2_In2, GPIO.OUT)
        self.pwm_2 = GPIO.PWM(self.Ena_M2, 100)
        self.pwm_2.start(0)
        
        # Set optical encoders
        
        GPIO.setup(self.En_1_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.En_1_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.En_2_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.En_2_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    def reset_encoders(self):
        '''
        Initialize encoder count and its previous state (set to -1 before a movement) before executing a movement
        '''
        # Set speed and duration to adjust perfect turns
        self.m1A_state_previous = -1
        self.m1B_state_previous = -1
        self.m2A_state_previous = -1
        self.m2B_state_previous = -1
        
        self.m1A_count, self.m1B_count = 0, 0
        self.m2A_count, self.m2B_count = 0, 0

    def increment_encoders(self):
        '''
        Read GPIO inputs of both encoders (each encoder gives two outputs)
        If the state has changed then increment the count
        We will use this count to ensure both motors are in sync while moving straight
        And these counts will help us take perfect turns
        '''

        self.m1A_state = GPIO.input(self.En_1_A)
        self.m1B_state = GPIO.input(self.En_1_B)
        if self.m1A_state != self.m1A_state_previous:
            self.m1A_state_previous = self.m1A_state
            self.m1A_count += 1
            self.m1B_state_previous = self.m1B_state
            self.m1B_count += 1

        self.m2A_state = GPIO.input(self.En_2_A)
        self.m2B_state = GPIO.input(self.En_2_B)
        if self.m2A_state != self.m2A_state_previous:
            self.m2A_state_previous = self.m2A_state
            self.m2A_count += 1
            self.m2B_state_previous = self.m2B_state
            self.m2B_count += 1

    def wait(self):
        time.sleep(15)

    def straight(self, duration = 3, speed = 70):
        
        m1_speed, m2_speed = speed, speed
        t_end = time.time() + duration
        
        self.reset_encoders()
        
        while time.time() < t_end:
            GPIO.output(self.M1_In1, GPIO.HIGH)
            GPIO.output(self.M1_In2, GPIO.LOW)
            self.pwm_1.ChangeDutyCycle(m1_speed)
                
            GPIO.output(self.M2_In1, GPIO.LOW)
            GPIO.output(self.M2_In2, GPIO.HIGH)
            self.pwm_2.ChangeDutyCycle(m2_speed)
                
            self.increment_encoders()
            diff = self.m1A_count - self.m2A_count  # Difference between the encoder counts

            if abs(diff) > self.correction_threshold:
                # Increase_speed of motor that is moving slow and vice versa
                if diff < 0:
                    m2_speed = min(90, m2_speed + 0.25 * abs(diff))
                    m1_speed = max(50, m1_speed -  0.25 * abs(diff))
                    
                elif diff > 0:
                    m1_speed = min(90, m1_speed +  0.25 * abs(diff))
                    m2_speed = max(50, m1_speed -  0.25 * abs(diff))
                
                # reset difference in count
                self.m1A_count = self.m2A_count
                #print(f'{m1_speed}, {m2_speed}')
                    
        # Stop motors
        self.pwm_1.ChangeDutyCycle(0)
        self.pwm_2.ChangeDutyCycle(0)
        #print(f'M1A: {self.m1A_count}, M1B: {self.m1B_count}\nM2A: {self.m2A_count}, M2B: {self.m2B_count}')
        time.sleep(1)
    
    def reverse(self, duration = 3, speed = 70):
        m1_speed, m2_speed = speed, speed
        t_end = time.time() + duration
        
        self.reset_encoders()
        while time.time() < t_end:
            GPIO.output(self.M1_In1, GPIO.LOW)
            GPIO.output(self.M1_In2, GPIO.HIGH)
            self.pwm_1.ChangeDutyCycle(speed)
            GPIO.output(self.M2_In1, GPIO.HIGH)
            GPIO.output(self.M2_In2, GPIO.LOW)
            self.pwm_2.ChangeDutyCycle(speed)
            
            self.increment_encoders()
            diff = self.m1A_count - self.m2A_count  # Difference between the encoder counts

            if abs(diff) > self.correction_threshold:
                # Increase_speed of motor that is moving slow
                if diff < 0:
                    m2_speed = min(90, m2_speed + 0.25 * abs(diff))
                    m1_speed = max(50, m1_speed -  0.25 * abs(diff))
                    
                elif diff > 0:
                    m1_speed = min(90, m1_speed +  0.25 * abs(diff))
                    m2_speed = max(50, m1_speed -  0.25 * abs(diff))
                
                self.m1A_count = self.m2A_count
                #print(f'{m1_speed}, {m2_speed}')
            
        # Stop motors
        self.pwm_1.ChangeDutyCycle(0)
        self.pwm_2.ChangeDutyCycle(0)
        time.sleep(1)
        
    def right(self, angle = 90, speed = 50):
        
        counts = int((80*angle)/9)

        self.reset_encoders()
        
        while max(self.m1A_count, self.m2A_count)/2 <= counts:
            GPIO.output(self.M1_In1, GPIO.LOW)
            GPIO.output(self.M1_In2, GPIO.HIGH)
            self.pwm_1.ChangeDutyCycle(speed)
                
            GPIO.output(self.M2_In1, GPIO.LOW)
            GPIO.output(self.M2_In2, GPIO.HIGH)
            self.pwm_2.ChangeDutyCycle(speed)

            self.increment_encoders()
            
        # Stop motors
        self.pwm_1.ChangeDutyCycle(0)
        self.pwm_2.ChangeDutyCycle(0)
        #print(f'M1A: {m1A_count}, M1B: {m1B_count}\nM2A: {m2A_count}, M2B: {m2B_count}')
        time.sleep(1)
            
    def left(self, angle = 90, speed = 50):
        counts = int((80*angle)/9)
        
        self.reset_encoders()
        
        while max(self.m1A_count, self.m2A_count)/2 <= counts:
            GPIO.output(self.M1_In1, GPIO.HIGH)
            GPIO.output(self.M1_In2, GPIO.LOW)
            self.pwm_1.ChangeDutyCycle(speed)
                
            GPIO.output(self.M2_In1, GPIO.HIGH)
            GPIO.output(self.M2_In2, GPIO.LOW)
            self.pwm_2.ChangeDutyCycle(speed)
            self.increment_encoders()
                
        # Stop motors
        self.pwm_1.ChangeDutyCycle(0)
        self.pwm_2.ChangeDutyCycle(0)
        time.sleep(1)
        
    def stop(self):
        
        self.pwm_1.ChangeDutyCycle(0)
        self.pwm_2.ChangeDutyCycle(0)