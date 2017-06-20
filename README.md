# grbl-tools
A collection (WIP) of gcode and GRBL related scripts and tools

### autoprobe.py

I wrote this script to autolevel gcode files that I use for PCB etching. The generated JSON is compatible with Chilipeppr GUI.

##### Future work:
- RPC interface for remote probing
- Apply the probe results to the input gcode file
- Optimize gcode toolpaths to minimize distance (TSP problem)
