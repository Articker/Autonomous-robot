This is source code(raw - needs some cleaning) of a program for my systems enginnering thesis: Complex mobile robot system capable of AR marker identification. The goal was to design and implement a system capable of controlling a mobile wheeled robot in such a way that it would reach the place of the predefined marker, used in augmented reality systems. Robot should navigate to AR marker and position itself infront of the marker (180 deegrees between camera surface and marker surface).

File rpiServer.py is for running at Raspberry pi to communicate PC and Arduino programs and it should be launch first. 
Main_program.py is a control system with image recognition using openCV. It should be launch second. IP adresses must be set before starting scripts (rpiServer.py and Main_program.py).
Arduino_code.c should by change to .ino file and set in Arduino platform. Schema of the circute and used devices is simple: 2 motors with incremental encoders and l293D shield for H bridge function. Pins and connections are described in code. 

Python scripts need libraries: Opencv,ZMQ, Imutils, Numpy, Keyboard (could be removed - was implemented only for manual control additional tests). 
