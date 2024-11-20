import serial
import time
import csv
import sympy as sp

# Define the trilateration algorithm
def trilateration(d_A, d_B, d_C, d_D):
    # Define the variables
    x, y, z = sp.symbols('x y z')

    # Define the equations for the distances to each anchor
    eq1 = x**2 + y**2 + z**2 - d_A**2
    eq2 = (x - 2.5)**2 + y**2 + z**2 - d_B**2
    eq3 = (x - 2.5)**2 + (y - 2.5)**2 + z**2 - d_C**2
    eq4 = x**2 + (y - 2.5)**2 + z**2 - d_D**2

    # Solve for x using eq1 and eq2
    eq_x = sp.simplify(eq2 - eq1)
    sol_x = sp.solve(eq_x, x)[0]

    # Solve for y using eq1 and eq4
    eq_y = sp.simplify(eq4 - eq1)
    sol_y = sp.solve(eq_y, y)[0]

    # Substitute the solutions for x and y into eq1 to solve for z
    # eq_for_z = eq1.subs([(x, sol_x), (y, sol_y)])
    # sol_z = sp.simplify(sp.solve(eq_for_z, z)[0])

    # Return the computed x, y, z coordinates, ensuring z is positive
    return float(sol_x), float(sol_y)#, abs(float(sol_z))

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
serialCom.flushInput()  # Flush input

# Open the CSV file to write data
file_name = "csv_algo.csv"
with open(file_name, mode='w', encoding='UTF-8', newline='') as filecsv:
    csv_writer = csv.writer(filecsv)
    
    # Write a header row with a timestamp and the position columns
    csv_writer.writerow(["Time", "Anchor1", "Anchor2", "Anchor3", "Anchor4", "Est_X", "Est_Y"])

    try:
        while True:
            if serialCom.in_waiting > 0:
                # Read the line from serial and decode it
                line = serialCom.readline().decode('utf-8').strip()
                print(line)  # Print for debug purposes
                
                # Split the data assuming comma-separated values
                data = line.split(',')

                if len(data) == 4:  # Ensure we have four distance values
                    try:
                        # Convert the distances to float values
                        d_A, d_B, d_C, d_D = [float(dist) for dist in data]

                        # Get the current time (hours, minutes, and seconds)
                        current_time = time.strftime('%H:%M:%S', time.localtime())

                        # Use the trilateration method to compute the position
                        x, y = trilateration(d_A, d_B, d_C, d_D)

                        # Write the time, distances, and estimated position to CSV
                        csv_writer.writerow([current_time, d_A, d_B, d_C, d_D, "{:.2f}".format(x), "{:.2f}".format(y)]) #, "{:.2f}".format(z)])

                        # Print out the estimated position
                        print(f"Estimated Position: X={x:.2f}, Y={y:.2f}")

                    except Exception as e:
                        print(f"Error in trilateration calculation: {e}")
                else:
                    print("Error: Line does not contain 4 values, skipping.")
            
    except KeyboardInterrupt:
        print("Serial reading interrupted.")

# Close the serial port when done
serialCom.close()
