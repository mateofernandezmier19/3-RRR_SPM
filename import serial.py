import serial

# Open a serial connection to the Arduino
ser = serial.Serial('COM5', 9600)  # Change 'COM3' to the appropriate serial port on your computer

# Specify the path to the text file you want to send
file_path = 'ON'  # Change this to your file's path

ser.write(file_path.encode('utf-8'))


# Close the serial connection
ser.close()