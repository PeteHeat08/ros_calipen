# ros_kalipen
## Bluetooth connect
First, make sure the PC Bluetooth device is running. If the pen is on it should be detected with Bluetooth device as "Kalibracijska olovka". You can pair with the device (Calibration pen) using desktop settings. If this is not working check this [link](https://askubuntu.com/questions/248817/how-to-i-connect-a-raw-serial-terminal-to-a-bluetooth-connection). Once you pair with the device, in a terminal run:
```
sudo rfcomm connect /dev/rfcomm0 80:64:6F:C4:78:BE 1 &
```

This will start the communication with the Calibration pen (MAC 80:64:6F:C4:78:BE), and open a serial port /dev/rfcomm0. Make sure you add your user account to the dialout group, which has access permission to /dev/rfcomm0. If you are lazy, run:
```
sudo chmod -R 777 /dev/rfcomm0
```
## Rosrun
A single node in the package connects to the serial port and listens to 'P' and 'N' messages from the pen. It uses a std_msgs/Joy topic to push button-press-notifications (1 or 0). Once you have successfully started /dev/rfcomm0, rosrun the joy_node.py, which publishes to the joy topic by default. 
