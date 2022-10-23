import serial

serialPort = serial.Serial(port = "COM4", baudrate=38400, bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE)

serialString = ""                           # Used to hold data coming over UART

while(1):
    #try:
        # Wait until there is data waiting in the serial buffer
        if(serialPort.in_waiting > 0):
            # Read data out of the buffer until a carraige return / new line is found
            serialString = serialPort.read_all().decode()
            # Print the contents of the serial data
            print(serialString)
    #except SerialError
