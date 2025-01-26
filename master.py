import serial
from serial import Serial
import serial.tools.list_ports as ports_
import time


def find_com_port():
    ports = list(prt.device for prt in ports_.comports())
    print(f"Available COM ports:{ports}")
    for port in ports:
        if "usbmodem" in port:
            return port
    print(f"NO usbmodem port found, COM available are:{ports}")
    raise ValueError("Invalid COM port")
    
    
def set_servo_angle(angle):
    print(f"Setting servo angle to {angle} degrees")
    if angle >= 0 and angle <= 180:
        arduino.write(f"{angle}".encode())  # Send the angle as a newline-terminated string
        response = arduino.readline().decode().strip()  # Read Arduino's response
        print(response)
    else:
        print("Angle out of range (0-180)")
        
def set_joints_angles(angles: list):
    message = ','.join(map(str, angles))
    print(f"Setting servo angles to {message} degrees")
    arduino.write(message.encode('utf-8'))
    arduino.write(b'\n')  # Send newline as a terminator    
    response = arduino.readline().decode().strip()  # Read Arduino's response
    print(response)

if __name__ == "__main__":    
    com = find_com_port()
    arduino = serial.Serial(port=com, baudrate=9600)
    
    continue_ = True
    while arduino.isOpen() and continue_:
        time.sleep(2) # Wait for connection to happen

        #   left_back_upper.write(80);
        #   left_back_lower.write(110);
        #   set_constrained_feet_pos(30, left_back_feet);

        #   right_back_upper.write(100);
        #   right_back_lower.write(70);
        #   set_constrained_feet_pos(60, right_back_feet);

        #   left_front_upper.write(90);
        #   left_front_lower.write(50);
        #   set_constrained_feet_pos(60, left_front_feet);

        #   right_front_upper.write(90);
        #   right_front_lower.write(130);
        #   set_constrained_feet_pos(30, right_front_feet);

        # set_joints_angles([80, 110, 30, 100, 70, 60, 90, 50, 60, 90, 130, 30])
        set_joints_angles([60, 90, 10, 100, 70, 60, 90, 50, 60, 90, 130, 30])
        
        req = input("Do you want to continue? (y/n)")
        if req.lower() == 'n':
            continue_ = False

    arduino.close()  # Close the serial connection
