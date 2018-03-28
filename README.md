# Stardust arduino

Arduino code - Stardust team - Eurobot 2018

This repository contains the code of the arduino boards.
Code has been written to communicate with a raspberry 3 board running ROS and a ROSSERIAL node.

## Installation process

On the raspberry pi :

1. Update SWAP to 1Go as ROS SERIAL compilation need 1,4 Go of RAM

```bash
sudo vi /etc/dphys-swapfile 
sudo /etc/init.d/dphys-swapfile restart
```

2. Compile ROS SERIAL

```bash
cd /home/pi/catkin_ws
source devel/setup.bash
cd src
git clone https://github.com/ros-drivers/rosserial.git
cd /..
catkin_make
catkin_make install
```

3. Update SWAP back to its previous value
```bash
sudo vi /etc/dphys-swapfile 
sudo /etc/init.d/dphys-swapfile restart
```

4. Configure arduino workspace

```bash
cd /home/pi
git clone https://github.com/julienbayle/stardust_arduino
cd stardust_arduino/motorboard/lib
rosrun rosserial_arduino make_libraries.py .
cd ../sensorboard/lib
rosrun rosserial_arduino make_libraries.py .
```

5. Install ino tool (Command line toolkit for Arduino)

```bash
sudo apt-get update 
sudo apt-get install vim arduino python-dev python-setuptools
git clone git://github.com/amperka/ino.git
cd ino
sudo python setup.py install (or make make install)
```

6. Compile and send code to the arduino boards using ino

```bash
cd /home/pi/stardust_arduino/motorboard
ino build
ino upload
rm .build -Rf
```

```bash
cd /home/pi/stardust_arduino/sensorboard
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

