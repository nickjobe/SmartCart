import time
import lgpio

'''Custom LED Driver created with ChatGPT assistance. Note: Our custom kernel did not support standard LCD drivers'''
class LCD:
    def __init__(self, chip_handle, rs, en, d4, d5, d6, d7, backlight):
        self.h = chip_handle
        self.LCD_RS = rs
        self.LCD_EN = en
        self.LCD_D4 = d4
        self.LCD_D5 = d5
        self.LCD_D6 = d6
        self.LCD_D7 = d7
        self.LCD_BACKLIGHT = backlight
        self.setup_gpio()
        self.lcd_init()
    
    def setup_gpio(self):
        """Initialize GPIOs for LCD."""
        for pin in [self.LCD_RS, self.LCD_EN, self.LCD_D4, self.LCD_D5, self.LCD_D6, self.LCD_D7, self.LCD_BACKLIGHT]:
            try:
                lgpio.gpio_claim_output(self.h, pin)
            except lgpio.error:
                print(f"Warning: GPIO {pin} is already in use. Skipping...")
    
    def lcd_init(self):
        """Initialize LCD in 4-bit mode."""
        self.lcd_command(0x33)  # Initialize
        self.lcd_command(0x32)  # Set to 4-bit mode
        self.lcd_command(0x28)  # 2 lines, 5x8 matrix
        self.lcd_command(0x0C)  # Display on, cursor off
        self.lcd_command(0x06)  # Entry mode
        self.lcd_command(0x01)  # Clear display
    
    def lcd_command(self, cmd):
        """Send command to LCD."""
        lgpio.gpio_write(self.h, self.LCD_RS, 0)  # Command mode
        self.send_nibble(cmd >> 4)  # Send high nibble
        self.send_nibble(cmd & 0x0F)  # Send low nibble
        time.sleep(0.001)
    
    def lcd_write(self, data):
        """Send character to LCD."""
        lgpio.gpio_write(self.h, self.LCD_RS, 1)  # Data mode
        self.send_nibble(data >> 4)  # Send high nibble
        self.send_nibble(data & 0x0F)  # Send low nibble
        time.sleep(0.001)
    
    def send_nibble(self, nibble):
        """Send 4-bit data to LCD."""
        lgpio.gpio_write(self.h, self.LCD_D4, (nibble >> 0) & 1)
        lgpio.gpio_write(self.h, self.LCD_D5, (nibble >> 1) & 1)
        lgpio.gpio_write(self.h, self.LCD_D6, (nibble >> 2) & 1)
        lgpio.gpio_write(self.h, self.LCD_D7, (nibble >> 3) & 1)
        self.pulse_enable()
    
    def pulse_enable(self):
        """Pulse the enable pin to latch data."""
        lgpio.gpio_write(self.h, self.LCD_EN, 1)
        time.sleep(0.0005)
        lgpio.gpio_write(self.h, self.LCD_EN, 0)
        time.sleep(0.0005)
    
    def lcd_message(self, line1, line2=""):
        """Display a message on the LCD (2 lines)."""
        self.lcd_command(0x01)  # Clear display
        for char in line1:
            self.lcd_write(ord(char))
        if line2:
            self.lcd_command(0xC0)  # Move to second line
            for char in line2:
                self.lcd_write(ord(char))
