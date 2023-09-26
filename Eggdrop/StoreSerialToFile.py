import serial
import csv

# serialPort = '/dev/cu.usbmodem1101' # "COM3"
serialPort = "COM6"
# Open the serial port (the COM port may vary, check the Arduino IDE for the correct port)
ser = serial.Serial(serialPort, 9600)

# Open or create a CSV file in write mode
with open('serial_data.csv', 'w', newline='') as f:
    writer = csv.writer(f)
    
    # Write the header row
    writer.writerow(['Timestamp', 'Target', 'MotorPWM', 'Encoder']) # Timestamp; Target; MotorPWM; Encoder;
    
    try:
        while True:
            # Read a line from the serial port
            line = ser.readline()
            print(line)
            # Decode the line to convert it from bytes to string
            line_str = line.decode('utf-8').strip()
            
            # Split the line at commas to create a list of values
            values = line_str.split(', ')
            
            # Write the values to the CSV file
            writer.writerow(values)
    except KeyboardInterrupt:
        # Exit the loop on a keyboard interrupt (Ctrl+C)
        pass
    finally:
        # Close the serial port
        ser.close()



# Arduino Code
'''
void setup() {
    Serial.begin(9600);
}

void loop() {
    unsigned long timestamp = millis();
    float voltage = analogRead(A0) * (5.0 / 1023.0);
    int position = analogRead(A1);
    
    Serial.print(timestamp);
    Serial.print(", ");
    Serial.print(voltage);
    Serial.print(", ");
    Serial.println(position);
    
    delay(1000);
}
'''