import serial #serial comms lib.

if __name__ == '__main__':   #sets address, comms speed of serial comm.
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    ser.flush()

    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').rstrip() #decodes machine gibberish into understandable data  
            print(line) #prints data
