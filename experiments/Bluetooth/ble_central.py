from bluepy.btle import Peripheral, UUID

PERIPHERAL_MAC = "2C:CF:67:72:B6:50"

def send_message():
    print(f"Connecting to {PERIPHERAL_MAC}...")
    try:
        central = Peripheral(PERIPHERAL_MAC)
        service = central.getServiceByUUID("12345678-1234-5678-1234-56789abcdef0")
        characteristic = service.getCharacteristics(UUID("abcdef01-1234-5678-1234-56789abcdef0"))[0]

        message = "Hello, Peripheral!"
        characteristic.write(message.encode(), withResponse=True)
        print(f"Sent: {message}")

        central.disconnect()

    except Exception as e:
        print(f"Connection failed: {e}")

if __name__ == "__main__":
    send_message()

