# BlueROV Project

This repo contains basic scripts for controlling a BlueROV and testing motor functions.  
It includes simple detection → motor activation, and a set of helper functions for moving the robot.

## Files
- **arm_and_test.py** – turns on the motor when it detects someone.  
- **controls.py** – functions responsible for moving the robot.  

## Requirements
- Python 3  
- GI / PyGObject library (install guide: https://pygobject.gnome.org/getting_started.html)  

If you see `ModuleNotFoundError: No module named 'gi'`, follow the link above and install the PyGObject library for your system.

## How to Run
### Detection + Motor Test

python arm_and_test.py




