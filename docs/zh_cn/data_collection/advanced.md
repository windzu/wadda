# Data Collection 进阶配置

## Usage

```bash
wadda dc [path] --pro
```

## Topic Setting

> 通过在配置文件中指定具体的topic，将只记录指定topic的数据

```yaml
topic: # must be topic
  topic_name: msg type
```

## Interval Setting

> 控制场景的采样时间和采样间隔

```yaml
constants:
  total_frames: 40 # Sample 40 frames per scene
  scene_interval: 0.5 # interval 0.5s
```

## Primary Key

> 指定某个frame_id 的时间戳作为时间同步的参考基准

```yaml
primary_frame_id: LIDAR_00
```

## Calibration

> 如果有多个激光雷达数据需要合并保存在一起，提供它们的外部参数，可以在采集数据的同时完成合并，减少采集的数据量

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