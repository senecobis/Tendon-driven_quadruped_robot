import serial
from serial import Serial
import serial.tools.list_ports as ports_
import time


def check_com(com):
    ports = list(prt.name for prt in ports_.comports())
    print(f"Available COM ports:{ports}")
    if com in ports and "cu" in com:
        return com
    else:
        print(f"Invalid COM port, COM available are:{ports}")
        raise ValueError("Invalid COM port")

# com = input("Enter the COM port: ")
# com = check_com(com)

com = "/dev/cu.usbmodem11101"
arduino = serial.Serial(port=com, baudrate=9600)
time.sleep(2)

def set_servo_angle(angle):
    print(f"Setting servo angle to {angle} degrees")
    if angle >= 0 and angle <= 180:
        arduino.write(f"{angle}".encode())  # Send the angle as a newline-terminated string
        time.sleep(0.1)
        response = arduino.readline().decode().strip()  # Read Arduino's response
        print(response)
    else:
        print("Angle out of range (0-180)")

# Test by moving the servo to different angles
for angle in range(0, 180, 10):
    set_servo_angle(angle)
    time.sleep(1)  # Wait 1 second between commands

arduino.close()  # Close the serial connection
