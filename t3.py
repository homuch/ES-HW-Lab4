import time
from bluepy.btle import Peripheral, UUID, DefaultDelegate, Scanner
from bluepy.btle import Characteristic
import threading
import binascii
# --- REPLACE THESE WITH YOUR ACTUAL VALUES ---
# TARGET_DEVICE_NAME = "Your BLE Device Name"  # The name of your BLE peripheral in nRF Connect
#SERVICE_UUID = UUID(0x1800)      # Replace with your service UUID
SERVICE_UUID = UUID("00000000-0001-11e1-9ab4-0002a5d5c51b")      # Replace with your service UUID
CHARACTERISTIC_UUID = UUID("00140000-0001-11e1-ac36-0002a5d5c51b") # Replace with the UUID of the characteristic that sends notifications/indications
CHAR_FOR_NOTIFY_UUID = UUID("00e00000-0001-11e1-ac36-0002a5d5c51b")
#CHARACTERISTIC_UUID = UUID("00140000-0001-11e1-ac36-0002a5d5c51b") # Replace with the UUID of the characteristic that sends notifications/indications

def int_to_binary_string(integer):
    """Converts an integer to a binary string in the format "b"\x01.....".

    Args:
        integer: The integer to convert.

    Returns:
        The binary string representation.
    """

    binary_string = bin(integer)[2:]  # Remove the '0b' prefix
    binary_string  = "0"*(32-len(binary_string)) + binary_string
    padded_binary = binary_string.zfill(8 * (len(binary_string) // 8 + 1))  # Pad to the nearest byte boundary
    bytes_list = [bytes([int(padded_binary[i:i+8], 2)]) for i in range(0, len(padded_binary), 8)]
    return b"".join(bytes_list)

def solve_x_axes(buff):
    """
    Extracts x_axes values from a buffer based on the provided macro and placements.

    Args:
        buff: A list or bytearray representing the buffer.

    Returns:
        A dictionary containing the x_axes values (AXIS_X, AXIS_Y, AXIS_Z).
        Returns None if the buffer is too short.
    """

    if len(buff) < 8:  # Check for sufficient buffer length
        return None

    x_axes= {}

    # Extract values using little-endian conversion and the given offsets
    x_axes["AXIS_Y"] = -int.from_bytes(buff[2:4], byteorder='little', signed=True)  # Note the negative sign for AXIS_Y and AXIS_Z
    x_axes["AXIS_X"] = int.from_bytes(buff[4:6], byteorder='little', signed=True)
    x_axes["AXIS_Z"] = -int.from_bytes(buff[6:8], byteorder='little', signed=True)

    return x_axes


def notification_loop(peripheral, characteristic):
    while True:
        time.sleep(0.2)
        print('', end='', flush=True)
        try:
            if peripheral.waitForNotifications(1.0):
                continue
        except Exception as e:
            print(f"error:{e}")
            pass
    #while True:
     #   time.sleep(0.1)
      #  val= characteristic.read().hex()
       # x_axes = solve_x_axes( buff = binascii.unhexlify(val))
        #print(f"\033[2mread {x_axes}\033[0m", flush=True)



def interactive_input(peripheral, characteristic):
    while True:
        #new_sf=2000
        num=input("input the frequency you like:")
        num = int(num)
        #data_to_write = (b"\x00\x00\x07\xd0")[::-1]
        data_to_write=int_to_binary_string(num)[::-1]
        characteristic.write(data_to_write, withResponse=True)
        print(f"Data {data_to_write} written successfully")
        # Now *wait* for the notification.  Don't write in a tight loop.



class MyDelegate(DefaultDelegate):
    def __init__(self):
        DefaultDelegate.__init__(self)

    def handleNotification(self, cHandle, data):
        val= data.hex()
        x_axes = solve_x_axes( buff = binascii.unhexlify(val))
        start='\033[96m'
        ENDC = '\033[0m'
        print(f"{start}Received notification: {x_axes} (handle: {cHandle}){ENDC}")



def connect_and_subscribe(device_addr):
    print(f"Connecting to {device_addr}...")
    try:
        p = Peripheral(device_addr, 'random')  # No timeout here, let it handle connection issues
        p.withDelegate(MyDelegate())

        print("Discovering services...")
        services = p.getServices()
        for service in services:
            print(f"service: {service}")
            if service.uuid == SERVICE_UUID:
                print(f"Found service: {service.uuid}")
                characteristics = service.getCharacteristics(forUUID=CHARACTERISTIC_UUID)
                if characteristics:
                    characteristic = characteristics[0]  # Assuming one characteristic with this UUID
                    print(f"Found characteristic: {characteristic.uuid}, handle: {characteristic.getHandle()}")


                    print(f"properties:{characteristic.properties}")
                    print(f"all propertis in CHAR....:{Characteristic.props}")
                    support_notify=characteristic.properties & Characteristic.props["NOTIFY"]
                    support_indicate=characteristic.properties & Characteristic.props["INDICATE"]
                    support_write=characteristic.properties & Characteristic.props["WRITE"]
                    support_write_no_resp=characteristic.properties & Characteristic.props["WRITE_NO_RESP"]

                    char_notif = service.getCharacteristics(forUUID=CHAR_FOR_NOTIFY_UUID)[0]
                    if support_notify or support_indicate or support_write or support_write_no_resp: # check notify, indicate, write, write without response property
                        if characteristic.supportsRead():
                            print(f"initial value:{characteristic.read().hex()}")

                        cccd_handle = characteristic.getHandle() + 1
                        print(f"CCCD handle: {cccd_handle}")

                        # Enable notifications/indications
                        if support_indicate: # check indicate property
                            p.writeCharacteristic(cccd_handle, b"\x02\x00", withResponse=True)
                        characteristic_handle = char_notif.getHandle()
                        cccd_handle = characteristic_handle + 1  # Usually, but check documentation
                        p.writeCharacteristic(cccd_handle, b"\x01\x00", withResponse=True)
                        notification_thread = threading.Thread(target=notification_loop, args=(p, char_notif), daemon=True) # daemon thread will close when the main thread exits
                        notification_thread.start()

                        interactive_thread = threading.Thread(target=interactive_input, args=(p, characteristic))
                        interactive_thread.start()


            # Main thread waits for the interactive thread to finish (when the user enters 'q')
                        #interactive_thread.join()

                        
                    else:
                        print(f"characteristic {CHARACTERISTIC_UUID} doesn't support notify, indicate, write or write without response property")

                else:
                    print(f"Characteristic {CHARACTERISTIC_UUID} not found in service {service.uuid}")

                return p  # Return the peripheral object to keep the connection alive


        print(f"Service {SERVICE_UUID} not found.")
        return None

    except Exception as e:
        print(f"Connection error: {e}")
        return None


class ScanDelegate(DefaultDelegate):
    def __init__(self):
        DefaultDelegate.__init__(self)

    def handleDiscovery(self, dev, isNewDev, isNewData):
        if isNewDev:
            print("Discovered device", dev.addr)
        elif isNewData:
            print("Received new data from", dev.addr)

def connect_get_addr():
    scanner = Scanner().withDelegate(ScanDelegate())
    devices = scanner.scan(5.0)
    n = 0
    addr = []
    mm = {}
    for dev in devices:
        print("%d: Device %s (%s), RSSI=%d dB" % (n, dev.addr, dev.addrType, dev.rssi))
        if dev.getValueText(9):
            mm[dev.getValueText(9)]=n
        addr.append(dev.addr)
        n += 1
        for (adtype, desc, value) in dev.getScanData():
            print(" %s = %s" % (desc, value))
    return "f6:8c:f2:d3:ea:e7"
    print(mm)
    number = input('Enter your device number: ')
    print('Device', number, addr[int(number)])
    num = int(number)
    # return addr[num]

def main():
    import sys
    try:
        addr = connect_get_addr()
        peripheral = connect_and_subscribe(addr)
        if peripheral:
            try:
                while True:
                    time.sleep(1)
                    # Add any other periodic tasks here if needed

            except KeyboardInterrupt:
                print("\nExiting...")
                sys.exit(0)

            finally:
                peripheral.disconnect()
                print("Disconnected.")

    except Exception as e:
        print(f"Error: {e}")


if __name__ == "__main__":
    while True:
        main()
