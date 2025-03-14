import lgpio
import time

# Define GPIO pins
LCD_RS = 25
LCD_EN = 24
LCD_D4 = 23
LCD_D5 = 17
LCD_D6 = 18
LCD_D7 = 22
LCD_BACKLIGHT = 4

# Initialize GPIO using lgpio
h = lgpio.gpiochip_open(0)  # Open GPIO chip

# Claim GPIO outputs
for pin in [LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7]:
    try:
        lgpio.gpio_claim_output(h, pin)
    except lgpio.error:
        print(f"Warning: GPIO {pin} is already in use. Skipping...")


# Function to send a command to the LCD
def lcd_command(cmd):
    lgpio.gpio_write(h, LCD_RS, 0)  # Command mode
    send_nibble(cmd >> 4)  # Send high nibble
    send_nibble(cmd & 0x0F)  # Send low nibble
    time.sleep(0.001)

# Function to send a character to the LCD
def lcd_write(data):
    lgpio.gpio_write(h, LCD_RS, 1)  # Data mode
    send_nibble(data >> 4)  # Send high nibble
    send_nibble(data & 0x0F)  # Send low nibble
    time.sleep(0.001)

# Function to send a 4-bit nibble
def send_nibble(nibble):
    lgpio.gpio_write(h, LCD_D4, (nibble >> 0) & 1)
    lgpio.gpio_write(h, LCD_D5, (nibble >> 1) & 1)
    lgpio.gpio_write(h, LCD_D6, (nibble >> 2) & 1)
    lgpio.gpio_write(h, LCD_D7, (nibble >> 3) & 1)
    pulse_enable()

# Function to pulse the enable pin
def pulse_enable():
    lgpio.gpio_write(h, LCD_EN, 1)
    time.sleep(0.0005)
    lgpio.gpio_write(h, LCD_EN, 0)
    time.sleep(0.0005)

# Initialize LCD
lcd_command(0x33)  # Initialize
lcd_command(0x32)  # Set to 4-bit mode
lcd_command(0x28)  # 2 lines, 5x8 matrix
lcd_command(0x0C)  # Display on, cursor off
lcd_command(0x06)  # Entry mode
lcd_command(0x01)  # Clear display

# Function to display a message on the LCD
def lcd_message(line1, line2=""):
    lcd_command(0x01)  # Clear display before writing
    time.sleep(0.002)  # Short delay to let the LCD clear

    for char in line1:
        lcd_write(ord(char))

    if line2:
        lcd_command(0xC0)  # Move cursor to second line
        for char in line2:
            lcd_write(ord(char))

# Display message
lcd_message("Hello", "SmartCart!")

# Keep running
while True:
    time.sleep(1)
