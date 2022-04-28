import serial, time

arduino = serial.Serial(port = 'COM3', baudrate = 9600, timeout=1)

while True:
    arduino.write(bytes("1", "utf-8"))
    arduino.write(bytes("2", "utf-8"))
    arduino.write(bytes("3", "utf-8"))

    if arduino.inWaiting():
        s = arduino.readline()

        s = s.decode()
        print(s.strip())

    time.sleep(0.01)

