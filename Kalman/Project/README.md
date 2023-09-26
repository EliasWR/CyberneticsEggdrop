# Python script
The python template script "arduino_send_receive.py" sends a single value to the Arduino and reads a string from the Arduino. Use this to receive sensor values from the Arduino, and send a value back to the Arduino, for example estimated height or motor voltage. This design choice is for you to make.

The script will try to communicate with an Arduino on 192.168.10.240, port 8888. 
Ensure you set a static IP with net mask 255.255.255.0 on the Ethernet adapter.


# Arduino sketch and libraries

Option 1)
Extract the two folders "fusion" and "libraries" to C:\Users\<USER>\Documents\Arduino

Option 2)
Alternatively, move only the folder "fusion" to the Arduino sketch folder, then open the Arduino IDE and the fusion sketch. From the menu toolbar, select "Sketch"->"Import library"->"Add .zip libraries" and import each of the three zips under "archives".