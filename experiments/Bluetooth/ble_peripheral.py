import gatt

class BLEPeripheral(gatt.DeviceManager):
    def __init__(self, adapter_name='hci0'):
        super().__init__(adapter_name)
    
    def device_discovered(self, device):
        print(f"Discovered: {device.mac_address} - {device.alias()}")

class CustomService(gatt.Service):
    UUID = "12345678-1234-5678-1234-56789abcdef0"

    def __init__(self, device):
        super().__init__(device, self.UUID, primary=True)

class CustomCharacteristic(gatt.Characteristic):
    UUID = "abcdef01-1234-5678-1234-56789abcdef0"

    def __init__(self, service):
        super().__init__(self.UUID, service, gatt.Characteristic.Properties.READ | gatt.Characteristic.Properties.WRITE)

    def ReadValue(self, options):
        return b"Hello from Peripheral!"

    def WriteValue(self, value, options):
        print(f"Received data: {value.decode()}")

def run_peripheral():
    print("Starting BLE Peripheral...")
    manager = BLEPeripheral(adapter_name='hci0')
    manager.run()

if __name__ == "__main__":
    run_peripheral()

