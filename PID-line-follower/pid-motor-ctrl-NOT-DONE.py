import serial #serial comms lib.

if __name__ == '__main__':   #sets address, comms speed of serial comm.
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    ser.flush()


    #might not work with uints? 
    while True:
        if ser.in_waiting > 0: #add global definitions
            pos = ser.readline().decode('utf-8').rstrip() #decodes machine gibberish into understandable data  
            err = ser.readline().decode('utf-8').rstrip()   #receives all of the values from arduino. Works by reading each line separately
            pidval = ser.readline().decode('utf-8').rstrip()
            
            print("line's position is: ", pos)
            print("error value is: ", err)
            print("pidvalue is: ", pidval)
    


