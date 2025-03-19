from machine import Pin, I2C
import time

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
        