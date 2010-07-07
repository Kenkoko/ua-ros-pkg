#!/bin/bash
echo "Plotting..."
rxplot /interface_kit/96952/sensor/0/data /interface_kit/96952/sensor/1/data /interface_kit/96952/sensor/2/data /interface_kit/96952/sensor/3/data /interface_kit/96952/sensor/4/data /interface_kit/96952/sensor/5/data  > output.dat
rm -f output.dat

