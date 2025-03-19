"""
EYEMECH ε3.1 control code adapted for ESP32 with PCA9685 servo controller
Based on Will Cogley's Eye Mechanism control code
"""

import time
from machine import Pin, I2C, ADC
import random
import sys  # Added for tracking initialization state

# PCA9685 constants
PCA9685_ADDRESS = 0x40
MODE1 = 0x00
MODE2 = 0x01
SUBADR1 = 0x02
SUBADR2 = 0x03
SUBADR3 = 0x04
PRESCALE = 0xFE
LED0_ON_L = 0x06
LED0_ON_H = 0x07
LED0_OFF_L = 0x08
LED0_OFF_H = 0x09
ALL_LED_ON_L = 0xFA
ALL_LED_ON_H = 0xFB
ALL_LED_OFF_L = 0xFC
ALL_LED_OFF_H = 0xFD

# Servo channel mapping on PCA9685
SERVO_CHANNELS = {
    "LR": 0,  # Left/Right movement
    "UD": 1,  # Up/Down movement
    "TL": 2,  # Top Left eyelid
    "BL": 3,  # Bottom Left eyelid
    "TR": 4,  # Top Right eyelid
    "BR": 5,  # Bottom Right eyelid
}

# Min, Max - possibly reversed for some eyelids
# Important: Ensure min is always the CLOSED position, max is OPEN position
servo_limits = {
   "LR": (40, 140),   
   "UD": (40, 140),
   "TL": (90, 10),   # Top Left eyelid (inverted)
   "BL": (10, 90),   # Bottom Left eyelid
   "TR": (10, 90),   # Top Right eyelid
   "BR": (90, 10),   # Bottom Right eyelid (inverted)
} 
    
class PCA9685:
    def __init__(self, i2c, address=PCA9685_ADDRESS):
        self.i2c = i2c
        self.address = address
        self.reset()
        
    def reset(self):
        self.write_byte(MODE1, 0x00)
        
    def write_byte(self, reg, value):
        self.i2c.writeto_mem(self.address, reg, bytes([value]))
        
    def read_byte(self, reg):
        return self.i2c.readfrom_mem(self.address, reg, 1)[0]
    
    def set_pwm_freq(self, freq_hz):
        """Set PWM frequency in Hz"""
        prescaleval = 25000000.0    # 25MHz
        prescaleval /= 4096.0       # 12-bit
        prescaleval /= float(freq_hz)
        prescaleval -= 1.0
        prescale = int(prescaleval + 0.5)
        
        oldmode = self.read_byte(MODE1)
        newmode = (oldmode & 0x7F) | 0x10    # sleep
        self.write_byte(MODE1, newmode)       # go to sleep
        self.write_byte(PRESCALE, prescale)
        self.write_byte(MODE1, oldmode)
        time.sleep_ms(5)
        self.write_byte(MODE1, oldmode | 0x80)  # restart
    
    def set_pwm(self, channel, on, off):
        """Sets PWM values for a specific channel"""
        self.i2c.writeto_mem(self.address, LED0_ON_L + 4 * channel, bytes([on & 0xFF]))
        self.i2c.writeto_mem(self.address, LED0_ON_H + 4 * channel, bytes([on >> 8]))
        self.i2c.writeto_mem(self.address, LED0_OFF_L + 4 * channel, bytes([off & 0xFF]))
        self.i2c.writeto_mem(self.address, LED0_OFF_H + 4 * channel, bytes([off >> 8]))
        
    def set_servo_angle(self, channel, angle):
        """Convert angle (0-180) to PWM value and set"""
        # Map angle 0-180 to PWM duty cycle
        # Standard servo values: 150 (0°) to 600 (180°) for pulse width
        pulse_width = int(150 + (angle * 450 / 180))
        self.set_pwm(channel, 0, pulse_width)

# Set up ESP32 I2C and PCA9685
i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=100000)
pca = PCA9685(i2c)
pca.set_pwm_freq(50)  # 50Hz is standard for servos

# Set up the switches and potentiometers on ESP32
enable = Pin(13, Pin.IN, Pin.PULL_UP)
mode = Pin(14, Pin.IN, Pin.PULL_UP)
blink_pin = Pin(15, Pin.IN, Pin.PULL_UP)

# ESP32 ADC setup
UD = ADC(Pin(34))
UD.atten(ADC.ATTN_11DB)  # Full voltage range (0-3.3V)
trim = ADC(Pin(35))
trim.atten(ADC.ATTN_11DB)
LR = ADC(Pin(32))
LR.atten(ADC.ATTN_11DB)

#Variables to track eye position and limit movement speed
last_lr_angle = 90
last_ud_angle = 90
max_speed = 5  # Maximum degrees change per cycle

# Set all servos to central position for assembly
def calibrate():
    # Center the eye movement servos
    pca.set_servo_angle(SERVO_CHANNELS["LR"], 90)
    pca.set_servo_angle(SERVO_CHANNELS["UD"], 90)
    
    # Close the eyelids
    pca.set_servo_angle(SERVO_CHANNELS["TL"], servo_limits["TL"][0])  # CLOSED position
    pca.set_servo_angle(SERVO_CHANNELS["BL"], servo_limits["BL"][0])  # CLOSED position
    pca.set_servo_angle(SERVO_CHANNELS["TR"], servo_limits["TR"][0])  # CLOSED position
    pca.set_servo_angle(SERVO_CHANNELS["BR"], servo_limits["BR"][0])  # CLOSED position

# Initialize controller mode - eyes centered, eyelids open
def initialize_controller_mode():
    # Center the eye movement servos
    pca.set_servo_angle(SERVO_CHANNELS["LR"], 90)
    pca.set_servo_angle(SERVO_CHANNELS["UD"], 90)
    
    # OPEN the eyelids
    pca.set_servo_angle(SERVO_CHANNELS["TL"], servo_limits["TL"][1])  # OPEN position
    pca.set_servo_angle(SERVO_CHANNELS["BL"], servo_limits["BL"][1])  # OPEN position
    pca.set_servo_angle(SERVO_CHANNELS["TR"], servo_limits["TR"][1])  # OPEN position
    pca.set_servo_angle(SERVO_CHANNELS["BR"], servo_limits["BR"][1])  # OPEN position
    print("Controller mode initialized")

# Neutral pose - eyes centered, eyelids open
def neutral():
    # Center eyes
    pca.set_servo_angle(SERVO_CHANNELS["LR"], 90)
    pca.set_servo_angle(SERVO_CHANNELS["UD"], 90)
    
    # Open eyelids
    pca.set_servo_angle(SERVO_CHANNELS["TL"], servo_limits["TL"][1])  # OPEN position
    pca.set_servo_angle(SERVO_CHANNELS["BL"], servo_limits["BL"][1])  # OPEN position
    pca.set_servo_angle(SERVO_CHANNELS["TR"], servo_limits["TR"][1])  # OPEN position
    pca.set_servo_angle(SERVO_CHANNELS["BR"], servo_limits["BR"][1])  # OPEN position

# Corrected blink function - should CLOSE the eyes
def blink():
    # Close all eyelids (meeting in the middle)
    pca.set_servo_angle(SERVO_CHANNELS["TL"], servo_limits["TL"][0])  # CLOSED position
    pca.set_servo_angle(SERVO_CHANNELS["BL"], servo_limits["BL"][0])  # CLOSED position
    pca.set_servo_angle(SERVO_CHANNELS["TR"], servo_limits["TR"][0])  # CLOSED position
    pca.set_servo_angle(SERVO_CHANNELS["BR"], servo_limits["BR"][0])  # CLOSED position

def scale_potentiometer(pot_value, servo, reverse=False):
    # ESP32 ADC range is 0-4095 (12-bit)
    in_min = 0
    in_max = 4095
    out_min = servo_limits[servo][0]
    out_max = servo_limits[servo][1]
    
    # Define center points with calibration offsets
    # These should be adjusted based on your actual joystick readings
    if servo == "LR":
        center_value = 2960  # Adjust if eyes look right/left when centered
    elif servo == "UD":
        center_value = 2970  # Adjust if eyes look up/down when centered
    else:
        center_value = 2048  # Default center
    
    # Create a small deadzone around center (±100)
    deadzone = 100
    if abs(pot_value - center_value) < deadzone:
        return 90  # Return exact center position
    
    # Scale based on which half of the range we're in
    if pot_value < center_value:
        # Lower half of range
        scaled_value = out_min + (pot_value - in_min) * (90 - out_min) / (center_value - deadzone - in_min)
    else:
        # Upper half of range
        scaled_value = 90 + (pot_value - (center_value + deadzone)) * (out_max - 90) / (in_max - (center_value + deadzone))
    
    if reverse:
        scaled_value = out_min + out_max - scaled_value
        
    # Clamp to valid range
    scaled_value = max(out_min, min(out_max, scaled_value))
        
    return int(scaled_value)

def scale_controller_potentiometer(pot_value, servo, reverse=False):
    """
    Special scaling function for controller mode with reduced movement range
    """
    # ESP32 ADC range is 0-4095 (12-bit)
    in_min = 0
    in_max = 4095
    
    # Define tighter movement limits for controller mode
    controller_limits = {
        "LR": (60, 120),  # More restricted range
        "UD": (60, 120),  # More restricted range
    }
    
    # Use the controller limits for LR and UD, fall back to normal limits for others
    if servo in controller_limits:
        out_min, out_max = controller_limits[servo]
    else:
        out_min, out_max = servo_limits[servo][0], servo_limits[servo][1]
    
    # Define center points with calibration offsets
    if servo == "LR":
        center_value = 2960  # Adjust based on your joystick
    elif servo == "UD":
        center_value = 2970  # Adjust based on your joystick
    else:
        center_value = 2048  # Default center
    
    # Create a small deadzone around center (±100)
    deadzone = 100
    if abs(pot_value - center_value) < deadzone:
        return 90  # Return exact center position
    
    # Scale based on which half of the range we're in
    if pot_value < center_value:
        # Lower half of range
        scaled_value = out_min + (pot_value - in_min) * (90 - out_min) / (center_value - deadzone - in_min)
    else:
        # Upper half of range
        scaled_value = 90 + (pot_value - (center_value + deadzone)) * (out_max - 90) / (in_max - (center_value + deadzone))
    
    if reverse:
        scaled_value = out_min + out_max - scaled_value
        
    # Clamp to valid range
    scaled_value = max(out_min, min(out_max, scaled_value))
        
    return int(scaled_value)

def control_ud_and_lids(ud_angle):
    """
    Improved function to move UD servo and make eyelids follow based on UD's position
    """
    # Get limits
    ud_min, ud_max = servo_limits["UD"]
    tl_min, tl_max = servo_limits["TL"]  # min=closed, max=open for TL
    tr_min, tr_max = servo_limits["TR"]  # min=closed, max=open for TR
    bl_min, bl_max = servo_limits["BL"]  # min=closed, max=open for BL
    br_min, br_max = servo_limits["BR"]  # min=closed, max=open for BR

    # Normalize UD position to a 0-1 range (0 = looking down, 1 = looking up)
    ud_range = ud_max - ud_min
    ud_progress = (ud_angle - ud_min) / ud_range if ud_range > 0 else 0.5
    
    # Calculate how much to close the eyelids based on looking up/down
    # When looking up, close top lids slightly
    # When looking down, close bottom lids slightly
    top_close_factor = 0.6 * (1 - ud_progress)  # Higher when looking down
    bottom_close_factor = 0.6 * ud_progress     # Higher when looking up
    
    # Find target positions - interpolate between open and partially closed
    tl_target = tl_min + (tl_max - tl_min) * (1 - top_close_factor)
    tr_target = tr_min + (tr_max - tr_min) * (1 - top_close_factor)
    bl_target = bl_min + (bl_max - bl_min) * (1 - bottom_close_factor)
    br_target = br_min + (br_max - br_min) * (1 - bottom_close_factor)
   
    # Move all servos
    pca.set_servo_angle(SERVO_CHANNELS["UD"], ud_angle)
    pca.set_servo_angle(SERVO_CHANNELS["TL"], int(tl_target))
    pca.set_servo_angle(SERVO_CHANNELS["TR"], int(tr_target))
    pca.set_servo_angle(SERVO_CHANNELS["BL"], int(bl_target))
    pca.set_servo_angle(SERVO_CHANNELS["BR"], int(br_target))

def update_eyelid_limits(trim_value):
    """
    Update eyelid limits based on trim potentiometer
    Ensures consistent behavior with min=closed, max=open
    """
    # Define a wider trim value range
    trim_min = 0
    trim_max = 4095
    
    # Define significant ranges for eyelid openness
    # For TL and BR (inverted servos):
    # - smaller values = more open
    # - higher values = more closed
    TL_range = (10, 80)  # From very open (10) to barely open (80)
    BR_range = (10, 80)  # From very open (10) to barely open (80)
    
    # For BL and TR:
    # - higher values = more open
    # - smaller values = more closed
    BL_range = (20, 90)  # From barely open (20) to very open (90)
    TR_range = (20, 90)  # From barely open (20) to very open (90)
    
    # Scale trim_value to determine how open eyelids should be
    trim_progress = (trim_value - trim_min) / (trim_max - trim_min)  # 0 to 1
    trim_progress = max(0, min(1, trim_progress))  # Clamp to [0, 1] range
    
    # Adjust MAX values (openness) based on trim
    # Keep MIN values fixed (closed position)
    servo_limits["TL"] = (90, TL_range[0] + (TL_range[1] - TL_range[0]) * (1-trim_progress))
    servo_limits["BR"] = (90, BR_range[0] + (BR_range[1] - BR_range[0]) * (1-trim_progress))
    servo_limits["BL"] = (10, BL_range[0] + (BL_range[1] - BL_range[0]) * trim_progress)
    servo_limits["TR"] = (10, TR_range[0] + (TR_range[1] - TR_range[0]) * trim_progress)
    
    # For debugging
    print(f"Trim: {trim_value}, Progress: {trim_progress}")
    print(f"Limits: TL:{servo_limits['TL']}, TR:{servo_limits['TR']}, BL:{servo_limits['BL']}, BR:{servo_limits['BR']}")

# Initialize PCA9685
print("Initializing servos...")
calibrate()
time.sleep_ms(1000)
print("System ready")

# Flag to track if controller mode has been initialized
controller_initialized = False

# Main loop
while True:
    mode_state = not mode.value()
    enable_state = not enable.value()
    
    if mode_state == 1:  # Calibration mode when switch is in hold position
        calibrate()
        time.sleep_ms(500)
        # Reset controller initialization flag when exiting controller mode
        controller_initialized = False
    else:
        if enable_state == 0:  # Auto mode
            # Reset controller initialization flag when exiting controller mode
            controller_initialized = False
            
            command = random.randint(0, 2)
            if command == 0:
                blink()
                time.sleep_ms(100)
                neutral()  # Return to neutral after blinking
                time.sleep_ms(random.randint(1000, 3000))
            elif command == 1:
                blink()
                time.sleep_ms(100)
                control_ud_and_lids(random.randint(servo_limits["UD"][0], servo_limits["UD"][1]))
                pca.set_servo_angle(SERVO_CHANNELS["LR"], random.randint(servo_limits["LR"][0], servo_limits["LR"][1]))
                time.sleep_ms(random.randint(300, 1000))
            elif command == 2:
                control_ud_and_lids(random.randint(servo_limits["UD"][0], servo_limits["UD"][1]))
                pca.set_servo_angle(SERVO_CHANNELS["LR"], random.randint(servo_limits["LR"][0], servo_limits["LR"][1]))
                time.sleep_ms(random.randint(200, 400))
        elif enable_state == 1:  # Controller mode
            # Initialize controller mode first time
            if not controller_initialized:
                initialize_controller_mode()
                controller_initialized = True
                
            # Reading sensors
            UD_value = UD.read()  # ESP32 ADC read() returns 0-4095
            trim_value = trim.read()
            LR_value = LR.read()
            blink_state = not blink_pin.value()
    
            update_eyelid_limits(trim_value)
            
            if blink_state == 0:  # Button pressed - blink (close eyes)
                blink()
            else:
                # Normal operation - move eyes according to joystick
                target_lr_angle = scale_controller_potentiometer(LR_value, "LR")
                target_ud_angle = scale_controller_potentiometer(UD_value, "UD")

                # Limit speed of movement
                if abs(target_lr_angle - last_lr_angle) > max_speed:
                    lr_angle = last_lr_angle + max_speed if target_lr_angle > last_lr_angle else last_lr_angle - max_speed
                else:
                    lr_angle = target_lr_angle

                if abs(target_ud_angle - last_ud_angle) > max_speed:
                    ud_angle = last_ud_angle + max_speed if target_ud_angle > last_ud_angle else last_ud_angle - max_speed
                else:
                    ud_angle = target_ud_angle

                # Update last positions
                last_lr_angle = lr_angle
                last_ud_angle = ud_angle

                # Apply movement
                pca.set_servo_angle(SERVO_CHANNELS["LR"], lr_angle)
                control_ud_and_lids(ud_angle)
                
            time.sleep_ms(10)
            print(f"LR: {LR.read()}, UD: {UD.read()}")
