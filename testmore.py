import serial
import time
import csv


# Open the serial port (replace with the correct port and baud rate)
try:
    serialCom = serial.Serial('/dev/ttyUSB0', 115200)
    print(f"Connected to: {serialCom.port}")
except serial.SerialException as e:
    print(f"Error connecting to serial port: {e}")
    exit()

# Reset the ESP32
serialCom.setRTS(True)  # Set RTS to True to reset
time.sleep(0.1)         # Small delay
serialCom.setRTS(False) # Set RTS to False to release the reset
serialCom.reset_input_buffer()  # Flush input

# Sample size
# sample = int(input("Enter the number of samples to collect: "))
# order = 0
# Open the CSV file to write data
file_name = "testmore.csv"
with open(file_name, mode='w', encoding='UTF-8', newline='') as filecsv:
    csv_writer = csv.writer(filecsv)
    
    # Write a header row with a timestamp column
    csv_writer.writerow(["Time", "Anchor1", "Anchor2", "Anchor3", "Anchor4"])

    try:
        while True: #order < sample: 
            if serialCom.in_waiting > 0:
                # Read the line from serial and decode it
                line = serialCom.readline().decode('utf-8').strip()
                print(line)  # Print for debug purposes
                
                # Split the data assuming comma-separated values
                data = line.split(',')
                
                if len(data) == 4:  # Ensure there are two values
                    # Get the current time (hours, minutes, and seconds)
                    current_time = time.strftime('%H:%M:%S', time.localtime())
                    
                    # Write the time and data to CSV
                    csv_writer.writerow([current_time] + data)
                else:
                    print("Error encountered, line was not recorded.")


                #order += 1
            
    except KeyboardInterrupt:
        print("Serial reading interrupted.")

# Close the serial port when done
serialCom.close()
