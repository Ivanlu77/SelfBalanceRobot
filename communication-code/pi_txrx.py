#!/usr/bin/env python3
import serial
import time

ser = serial.Serial(
    port='/dev/ttyAMA0',
    baudrate=115200,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    timeout=1
)

def main():
    try:
        print("Starting serial communication with ESP32...")
        while True:
            # Read data from ESP32
            if ser.in_waiting > 0:
                received_data = ser.readline().decode('utf-8').strip()
                print(f"Received from ESP32: {received_data}")
            
            # Send data to ESP32
            message = "Hello from Raspberry Pi!"
            ser.write((message + '\n').encode('utf-8'))
            print(f"Sent to ESP32: {message}")
            
            time.sleep(1)  # Wait for 1 second before next transmission
            
    except KeyboardInterrupt:
        print("\nExiting program...")
    except Exception as e:
        print(f"Error occurred: {e}")
    finally:
        ser.close()
        print("Serial port closed")

if __name__ == "__main__":
    main()
