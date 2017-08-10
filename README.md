# rpi-docker-ros-tensorflow

Now including ROS, tensorflow/models for Object Detection API and MobileNet for Detection

## Fork: romilly/rpi-docker-tensorflow

## Build instructions

1. Install Docker on your Raspberry Pi.
  1. `curl -sSL get.docker.com | sh`
  1. `sudo usermod -aG docker pi`
  1. log out, then log back in again for the change to take effect
  1. `sudo systemctl start docker`
1. Clone this repository into a directory of your choice
  1. `git clone https://github.com/esahin90/rpi-docker-ros-tensorflow.git`
1. Build the image
  1. `cd rpi-docker-tensorflow/build-tensor-pi/`
  1. `docker build -t='yourName/rpi-docker-tensorflow' .`

## Running the image for Detection and Web-Video-Server

`docker run -p 8080:8080 --device /dev/yourCamera yourName/ros-tensor`

## Running the image for Jupyter Notebook

This run instruction expects a directory called myNotebooks within your
home directory.

If you save an IPython notebook to the `myNotebooks` sub-directory
while running your container, it will get saved to the `myNotebooks`
directory on your Pi.

Notebooks saved to that directory will be _persistent_ - in other words,
they will still be there when the container is stopped and restarted.

`docker run -it -p 8888:8888 -v ~/myNotebooks:/notebooks/myNotebooks yourName/ros-tensorflow /run_jupyter.sh`

## Connecting to the notebooks

Open a browser on `http://raspberrypi:PORT` where raspberrypi is the
hostname of the Pi on which the docker image is running, or on
`http://localhost:PORT` on the Pi itself.

## Port

8080 - Web-Video-Server

8888 - Jupyter Notebook

## Stopping the image

`docker ps`
`docker stop containerID`

or

CTRL-C for Jupyter Notebook

# Sources

1. Docker: http://blog.alexellis.io/getting-started-with-docker-on-raspberry-pi/
1. [Base image](https://hub.docker.com/r/fjctp/armhf-ros-kinetic-base/): from fjctp/armf-ros-kinetic-base
1. [Pi tensorflow whl file](https://github.com/samjabrahams/tensorflow-on-raspberry-pi/raw/master/bin/tensorflow-1.1.0-cp27-none-linux_armv7l.whl)
from [Sam Abrahm's Github project](https://github.com/samjabrahams/tensorflow-on-raspberry-pi)
1. Notebooks and notebook config from [The Tensorflow Docker Build on Github](https://github.com/tensorflow/tensorflow/tree/master/tensorflow/tools/docker)
1. [Fork](https://github.com/romilly/rpi-docker-tensorflow): from romilly/rpi-docker-tensorflow
1. [SSD MobileNet v1 coco](https://github.com/tensorflow/models/blob/master/object_detection/g3doc/detection_model_zoo.md)
1. ROS Package: cv_camera, cv_bridge, web_video_server
