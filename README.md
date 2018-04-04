# Stardust arduino

Arduino code - Stardust team - Eurobot 2018

This repository contains the code of the arduino boards.
Code has been written to communicate with a raspberry 3 board running ROS and a ROSSERIAL node.

## Installation process

On the raspberry pi :

1. Install ROSSERIAL

```bash
sudo apt-get install ros-kinetic-rosserial
```

2. Configure arduino workspace

```bash
cd /home/pi
git clone https://github.com/julienbayle/starbaby_arduino
cd starbaby_arduino/motorboard
mkdir lib
cd lib
rosrun rosserial_arduino make_libraries.py .
cd ../sensorboard
ln -s /home/pi/starbaby_arduino/motorboard/lib lib
```

3. Install ino tool (Command line toolkit for Arduino)

```bash
sudo apt-get update 
sudo apt-get install vim arduino python-dev python-setuptools
git clone git://github.com/amperka/ino.git
cd ino
sudo python setup.py install (or make make install)
```

4. Compile and send code to the arduino boards using ino

```bash
cd /home/pi/starbaby_arduino/motorboard
ino build
ino upload
rm .build -Rf
```

```bash
cd /home/pi/starbaby_arduino/sensorboard
ino build
ino upload
rm .build -Rf
```

7. Test

```bash
roscore &
rosrun rosserial_python serial_node.py /dev/ttyACM0 &
rostopic list
```

All hardware topics should be listed
Use **rostopic pub** to change values and **rostopic echo** to read values 

