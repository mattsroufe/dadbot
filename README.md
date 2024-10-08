# dadbot

![](https://github.com/mattsroufe/dadbot/blob/master/IMG_2526.jpeg)

![](https://github.com/mattsroufe/dadbot/blob/master/screenshot.png)

## Demo video

https://youtu.be/T0yaw4qGdxE

## Run on raspberry pi

```
source /opt/ros/humble/setup.bash
cd ros2_ws
. install/setup.bash
ros2 launch dadbot bringup.launch.py
```

## Netwoking

Ensure Avahi is installed and running:
```
sudo systemctl status avahi-daemon
```
If not, either start the service using `sudo systemctl start avahi-daemon` or
install it with `sudo apt-get install avahi-daemon avahi-utils`

This allows you to connect using your robot's hostname on you local network like this:
```
ssh mattsroufe@dadbot2.local
```
And you don't need to worry about dynamic ip addresses changing on your network.

## Raspiberry pi setup

`sudo vim /boot/firmware/config.txt`

add the following two lines:
```
start_x=1 # enable raspi camera
gpu_mem=512
```
https://www.raspberrypi.org/documentation/configuration/config-txt/memory.md

random commands i've used:

```
sudo apt-get update
sudo apt-get upgrade

python3 --version
sudo apt-get install python3-dev python3-rpi.gpio

sudo service uv4l_raspicam start
sudo service uv4l_raspicam restart
sudo service uv4l_raspicam stop

sudo apt install fswebcam
fswebcam image.jpg

sudo nano /etc/uv4l/uv4l-raspicam.conf
uv4l --config-file=/etc/uv4l/uv4l-uvc.conf 

cat /etc/uv4l/uv4l-uvc.conf 
uv4l --config-file=/etc/uv4l/uv4l-uvc.conf --device-path=/dev/video0
uv4l --sched-rr --mem-lock --driver raspicam --width 960 --height 540 --framerate 30 --encoding mjpeg --vflip --hflip
uv4l --sched-rr --mem-lock --driver uvc --device-id=038f:6001

sudo service uv4l_raspicam status
lsusb

sudo vim /etc/systemd/system/uv4l_raspicam.service
sudo vim /etc/uv4l/uv4l-raspicam.conf

sudo apt install python3-pip
pip3 install aiortc
sudo apt-get install libffi
sudo apt install libavdevice-dev libavfilter-dev libopus-dev libvpx-dev pkg-config
sudo apt-get install python-cffi
sudo apt-get install python3-dev
sudo apt-get install -y python3-cffi
sudo apt install libsrtp2-dev
sudo apt-get install libatlas-base-dev
sudo apt-get install libjasper-dev
sudo apt-get install libqtgui4
sudo apt-get install python3-pyqt5
sudo apt-get install -y libilmbase-dev
sudo apt-get install -y libopenexr-dev
sudo apt-get install -y libgstreamer1.0-dev
sudo apt install libqt4-test

pip3 uninstall opencv-python
pip3 install opencv-python==3.4.6.27
```

## Development

```
cd frontend
npx webpack --mode development
python3 -m http.server
```

## ROS Resources

http://docs.donkeycar.com
https://www.youtube.com/playlist?list=PLjDc7gDlIASSp38zxcD_KMhogQBt7Dz_r linuxconf ros talks
