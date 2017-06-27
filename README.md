# grbl-tools
A collection (WIP) of gcode and GRBL related scripts and tools

### autoprobe.py

I wrote this script to autolevel gcode files that I use for PCB etching. The generated JSON is compatible with Chilipeppr GUI. At this moment the script is compatible with python2 and grbl 1.1f. Feel free to contribute!

##### Future work:
- Python3 compatibility
- RPC interface for remote probing
- Apply the probe results to the input gcode file
- Optimize gcode toolpaths to minimize distance (TSP problem)
