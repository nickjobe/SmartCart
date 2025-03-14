import socket
from lcdDriver import LCD
import lgpio

HOST = '0.0.0.0'
PORT = 5000

LCD_RS = 25
LCD_EN = 24
LCD_D4 = 23
LCD_D5 = 17
LCD_D6 = 18
LCD_D7 = 22
LCD_BACKLIGHT = 4

h = lgpio.gpiochip_open(0)

lcd = LCD(h, LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7, LCD_BACKLIGHT)
lcd.lcd_message("Server Ready", "Listening...")

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((HOST, PORT))
server_socket.listen(5)

print(f"Server listening on port {PORT}...")

while True:
    client_socket, client_address = server_socket.accept()
    print(f"Connection from {client_address} established.")
    
    data = client_socket.recv(1024).decode('utf-8')
    
    if data:
        print(f"Received: {data}")

        lcd.lcd_clear()
        lcd.lcd_message("Object Detected:", data[:16])

        client_socket.send("Message received".encode('utf-8'))
    
    client_socket.close()
