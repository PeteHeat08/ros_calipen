# ros_kalipen
## Bluetooth connect
First, make sure the PC Bluetooth device is running. If the pen is on it should be detected with Bluetooth device as "Kalibracijska olovka". You can pair with the device (Calibration pen) using desktop settings. If this is not working check this [link](https://askubuntu.com/questions/248817/how-to-i-connect-a-raw-serial-terminal-to-a-bluetooth-connection). Once you pair with the device, in a terminal run:
```
sudo rfcomm connect /dev/rfcomm0 80:64:6F:C4:78:BE 1 &
```

If `killall` command is not recognized install `psmisc` package as follows: 
```
sudo apt-get install psmisc
```

This will start the communication with the Calibration pen (MAC 80:64:6F:C4:78:BE), and open a serial port /dev/rfcomm0. Make sure you add your user account to the dialout group, which has access permission to /dev/rfcomm0. If you are lazy, run:
```
sudo chmod -R 777 /dev/rfcomm0

```
## Rosrun
A single node in the package connects to the serial port and listens to 'P' and 'N' messages from the pen. It uses a std_msgs/Joy topic to push button-press-notifications (1 or 0). Once you have successfully started /dev/rfcomm0, rosrun the joy_node.py, which publishes to the joy topic by default. 

Node uses `pyserial` package which can be installed with: `pip install pyserial` 

## Roslaunch 

After connecting to `kalibracijska olovka` you can run `roslaunch` which automatically starts 
rospy node as follows: 
```
roslaunch ros_kalipen start_ros_pen.launch 
```
Before roslaunch enable `/dev/rfcomm0` with `enable_kalipen.sh` script (NOTE: Uses sudo).  

## Bluetooth 

After turning off ROS node, or restarting PC run following: 
```
sudo bluetoothctl 
disconnect <mac_adress> 
scan on 
... wait for it to find Kalibracijska olovka ... 
scan off 
pair <mac_adress> 
... run enable_kalipen.sh in ROS pkg 
```




