import serial
import csv

ser = serial.Serial('/dev/ttyUSB0', 115200)
file_name = 'test.csv' # file name
print("Connected to: " + ser.portstr)
filecsv = open(file_name, 'w')
print("Created file: " + file_name)

sample = 5 # number of samples
print_labels = False
line = 0 # line counter, starts at 0
sensor_data = [] # list to store sensor data

#Collect sample and display the data in the terminal
while line < sample:
    getData = ser.readline()
    dataString = getData.decode('utf-8')
    data = dataString[0:][:-2]
    print(data)

    readings = data.split(', ')
    print(readings)

    sensor_data.append(readings)
    print(sensor_data)
    line = line + 1

#Create a csv file
with open(file_name, mode='w', encoding='UTF-8',newline='') as filecsv:
    writer = csv.writer(filecsv)
    writer.writerows(sensor_data)

    print("Data has been collected and saved in: " + file_name)
    filecsv.close()


