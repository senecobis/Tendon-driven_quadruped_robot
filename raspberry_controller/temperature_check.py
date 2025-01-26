from gipiozero import CPUTemperature
import time

continue_ = 'y'
while continue_ == 'y':
    for i in range(360):
        cpu = CPUTemperature()
        print(cpu.temperature)
        time.sleep(1000)
    continue_ = input("Do you want to continue? (y/n)")