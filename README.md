# Foscam R2 IP Camera Interface

This Docker image provides a ROS interface to the Foscam R2 IP surveillance camera.

## How to use it

Run it using the command:
```
docker run \
  -itd \
  --net=host \
  duckietown/dt-ttic-foscam-r2-interface:daffy
```

## Configuration files

If you have custom login information for your camera, the default image will not work out of the box, you will need to provide extra information.


### Configure login information

Create a directory `config` and create the following file `config/mycam.yaml`.
The content of this file has the following shape:
```
ip: 'STRING'
port: INT
username: 'STRING'
password: 'STRING'
framerate: INT
```

Mount the `/config` dir to your container and provide the new location for the config file.
```
docker run \
  -itd \
  --net=host \
  -v ./config:/config \
  -e CAMERA_PARAM_FILE=/config/mycam.yaml
  duckietown/dt-ttic-foscam-r2-interface:daffy
```


### Configure crop

The image provides a simple way to crop a given ROI from the camera image.
Cropping is enabled by default with the ROI set to be the entire image, so it has no effect.
If you want to configure the ROI, create a directory `config` and create the following file `config/myROI.yaml`.
The content of this file has the following shape:
```
x_offset: INT
y_offset: INT
width: INT
height: INT
```
For more information, check out [this page](http://wiki.ros.org/image_proc);

Mount the `/config` dir to your container and provide the new location for the config file.
```
docker run \
  -itd \
  --net=host \
  -v ./config:/config \
  -e CROP_PARAM_FILE=/config/myROI.yaml
  duckietown/dt-ttic-foscam-r2-interface:daffy
```


### Configure image rectification

By default, the image performs image rectification using the camera matrix in the `camera_info` topic. Set the environment variable `RECTIFY` to `0` to disable it.
