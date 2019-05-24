from serial import Serial
sp = Serial('/dev/ttyUSB0', 9600)

while True:
    sp.write(raw_input("your wish is my command: "))
