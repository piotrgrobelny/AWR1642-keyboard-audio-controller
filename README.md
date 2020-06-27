# AWR1642 audio and keyboard controller
Python application which makes audio and keyboard controller by using AWR1642 from TI and HB100 - simple arduino doppler radar.

### Requirements:
#### Hardware
- AWR1642 boost
- Arduino Uno
- HB100 doppler radar
#### Software
- Python 3 https://www.python.org/downloads/
- Arduino IDE https://www.arduino.cc/en/main/software
- pip https://pypi.org/project/pip/
## Installation process 
1.Connect HB100 to arduino
- VCC to 5V
- GND to GND
- Out to Analog output A0

2.Connect arduino and AWR1642 to PC by USB 

3.Place the AWR1642 board in an upright position with at least one meter of free space in front of the radar
  
4.Place HB100 radar in the way that it won't detect motion from other hand which will be controlled by AWR1632
  
5.Open `HB100.ino` file in `Arduino code` directory by Arduino IDE and upload following code to the board
  
6.Install python packages using `pip install`:
 - sys
 - numpy
 - serial
 - time
 - pyqtgraph 
 - PyQt5
 - pyo (only if you want to use audio controller)
 - keyboard (only if you want to use keyboard controller)
 
 7.Choose which controller you want to use and run python script:
 1. Audio controller:  
        -`radar_piano.py` - simulation of piano with note sounds from http://www.mediafire.com/file/zp8brp1f6q777b1/Midi_Notes.rar/file  
        -`radar_organ.py` - real time audio processing script which generates an organ sounds  
        
 2. Keyboard controller:  
        -`gui_app.py` - run keyboard controller with GUI  
        -`keyboard_controller` - run without GUI




 
