# Data Collection Advanced Config

## Topic Setting

> By specifying the topic in the configuration file, only the specified data can be recorded

```yaml
topic: # must be topic
  topic_name: msg type


```

## Interval Setting

> Controls the sampling time and sampling interval of a scene

```yaml
constants:
  total_frames: 40 # Sample 40 frames per scene
  scene_interval: 0.5 # interval 0.5s
```

## Primary Key

> Specifies the timestamp of a frame id as the synchronization reference

```yaml
primary_frame_id: LIDAR_00
```

## Calibration

> If there are multiple lidar data that need to be merged and saved together, provide their external parameters, and the merge can be completed while collecting data, reducing the amount of data collected

## Config Template

```yaml
topic:
  /CAM_00/compressed: sensor_msgs/CompressedImage
  /CAM_01/compressed: sensor_msgs/CompressedImage
  /CAM_02/compressed: sensor_msgs/CompressedImage
  /LIDAR_00: sensor_msgs/PointCloud2
  /LIDAR_01: sensor_msgs/PointCloud2
  /LIDAR_02: sensor_msgs/PointCloud2
  /LIDAR_03: sensor_msgs/PointCloud2

constants:
  total_frames: 10
  scene_interval: 0.5

primary_frame_id: LIDAR_00

calib:
  LIDAR_00 :
    rotation: [-0.012, 0.006, 0.385, 0.923]
    translation: [0.8648249,1.232774,-2.149898]
  LIDAR_01 :
    rotation: [0, 0, 0, 1]
    translation: [0,0,0]
  LIDAR_02 :
    rotation: [-0.003, -0.004, -0.362, 0.932]
    translation: [0.8926382,-1.304898,-2.194513]
  LIDAR_03 :
    rotation: [0.006, -0.694, 0.004, 0.720]
    translation: [-1.27779,-0.007783,-1.073833]
```