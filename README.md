# odrive_arduino

## rosserial velocity control

- Terminal 1
```bash
rosrun rosserial_python serial_node.py /dev/ttyACM0 _baud:=115200
```
- Terminal 2
```bash
rosrun odrive_odometry key_teleop.py
```

## rosserial encoder publish

- Terminal 1
```bash
rosrun rosserial_python serial_node.py /dev/ttyACM0 _baud:=115200
```
- Terminal 2
```bash
rosrun odrive_odometry key_teleop.py
```
- Terminal 3
```bash
rostopic echo /encoder
```
