# 📖 Developer documentation

## Files description
- [rollercoaster_task.py](../../src/rollercoaster_task.py) : The python source code for the two new megabot tasks. Both tasks require in their constructor a reference to the com, the robot, the robot lock. A bool given as fourth argument in the constructor describe if the task should use simulated (fake) telemetric data or not (for testing purpose)  
- [cartography.py](../../src/cartography.py) : Standalone python script allowing for an automatic sweep of the reachable positions for the centrale platform of the megabot (must be placed into the megabot control code source folder, next to *full.py*, to work correctly)
- [telemetry_receiver.py](../../src/telemetry_receiver.py) : Standalone python script reading the Epic Roller Quest telemetric data broadcast on the Local Area Network (can be used with teleplot to record roller coaster track movement as a .json file that can then be replayed as fake telemetric data)
- [simulated_telemetry.py](../../src/simulated_telemetry.py) : Auxiliary file, wrapper for the fake telemetric data .json reader 
- [washout_algorithm.py](../../src/washout_algorithm.py) : Auxiliary file, regrouping every filters and components of a washout filter

## Task description
Each main tasks is divided into 3 sections :
- The initialisation in the `__init__()` method. Contains all object and parameter initialisations
- The main loop in the `dtTick()` method. It usually contains a state machine composed of 3 states : `"start"`, `"running"` and `"end"`.  
`"start"` is for initialising time variables and get to the starting position.  
`"running"` is where the main movement or task takes place.  
`"end"` is optionnal and allows to return to a neutral position or save some data.  
- Other methods can be defined to be used in the main loop or the `__init__` method.


## Teleplot
Teleplot has been used thourought the project. This is a Visual Studio Code extension allowing for real time plotting of data. Like the telemetry data, teleplot data is exchanged via the Local Area Network. By default, the data is sent to ```teleplotAddr = ("127.0.0.1",47269)``` which is the device running the programm, on the port 47269. Please note that this port differs from the telemetry data which is by default 7701.

In order to plot the evolution of a variable over time
data packet sent to the teleplot address must respect this string format.
```<Variable Display Name>:<Timestamp>:<Value>|g```

A *sendTelemetry* function has been defined in most script of this project to standardize the sending of data to teleplot. It takes as argument the display name of the variable and its value, with the timestamp being automatically calculated.


# Telemetry Receiver
The telemetry receiver is defined twice for the two different tasks. It receives the data packets sent by the VR headset running Epic Roller Coaster. The data packet can be interpreted as a string.

For instance here is what a data packet typically looks like 
```S[003.665]V[017.797]F[014.206]Y[269.217]P[005.475]R[-03.000]H[-00.174]W[-00.316]U[03.050]A[-00.599]I[-00.303]O[-00.763]N[01]C[02]G[02]```

it can be decomposed into:
- Speed "S" (m/s)
- Vibration : Intensity "V", Frequency "F"
- Rotation : Yaw "P" Pitch "P" Roll "R" (degree)
- Linear Acceleration : Heave "H" Sway "S" Surge "U" (m/s/s)
- Angular Velocities : Yaw "A" Pitch "I" Roll "O" (radian/s)
- Ride information : Current ride "N" Current cart "C" Current gun "G" (indexes)

The telemetry receiver will block the task thread waiting for a packet. For the two RollerCoaster tasks, a time out has been set to 0.5s. Once the thread of the task has been blocked for more than this amount of time, it will continue with the previous telemetry values. Note that the task thread is different from the thread responsible for the control of the megabot. 
