# CyberneticsEggdrop
This is an NTNU School project in the subject of Cybernetics

___
## Arduino Code
#### Experiments and saving to file
The code in folder [MotorDrivingTest](MotorDrivingTest) is used to run experiments on the motor and logging useful data.

#### PID Control
The code in folder [PID_Controlled_Motor](PID_Controlled_Motor) is where the fun happens. That code is setup to drop an egg using a PID controller.
___
## Storing to file
To save useful data to a file, do the following:
- Configure the python file ["StoreSerialToFile.py"](StoreSerialToFile.py) to use the right COM-port
- Run that Python file 
- Run the Arduino Script [MotorDrivingTest](MotorDrivingTest)
- To stop the logger (the python script), make sure you are in the correct terminal and press 'ctrl+c' ('cmd+c' on mac)

The Arduino runs through a list of targets. Alter those as wanted.

___

## References
EGGDROP - @oystebje
https://github.com/oystebje/EGGDROP/tree/main
