# Data Collection

## 功能

将当前在广播的ros数据，按照固定的时间间隔保存至指定的路径下。其有如下特点

### 特点

* 每隔一段时间保存一帧数据，每满xx帧则新建另一个文件夹，继续保存

* 如果存在多个传感器数据需要录制，采样数据的时候会按照时间软同步的方式保存

## 使用场景

采集某个场景下的传感器数据时，如果直接通过ros录制bag包，bag包的体积会非常大，以至于难以传输和解析。而且bag包中的数据是连续性的，帧与帧之间的差别很小，信息冗余太高。

所以可以每隔一段时间保存一帧数据来达到减轻存储负担的目的。

此外，如果所采集的数据需被用于传感器的融合应用，还需要保证的数据同步性质，所以还需进行数据同步

## Simple Usage

* path：Specify the path where you want the data to be stored

```bash
wadda dc .
```

## Usage

```bash
wadda dc [path]
```

## 进阶使用

在简单使用中默认将保存所有所支持的保存的数据，保存数据的帧数等也都是默认的，在进阶使用中，可通过配置文件来做到更加自由的设置，其功能主要如下：

* 指定具体需要记录的topic

* 设置采样的时间和采样间隔

* 指定某个topic作为时间同步的基准

* 提供标定参数供lidar数据拼接，减少数据存储量

如下是一个config示例,config文件名需为config.yaml

```yaml
# 希望从众多topic中只记录如下的7个tipoc数据
topic:
  /CAM_00/compressed: sensor_msgs/CompressedImage
  /CAM_01/compressed: sensor_msgs/CompressedImage
  /CAM_02/compressed: sensor_msgs/CompressedImage
  /LIDAR_00: sensor_msgs/PointCloud2
  /LIDAR_01: sensor_msgs/PointCloud2
  /LIDAR_02: sensor_msgs/PointCloud2
  /LIDAR_03: sensor_msgs/PointCloud2

# 设置一个场景采样10帧，每隔0.5s采集一帧，即一个场景采样5s
constants:
  total_frames: 10
  scene_interval: 0.5

# 以frame_id为LIDAR_00的时间为同步的参考时钟
primary_frame_id: LIDAR_00

# 希望将四个lidar的点云数据通过外參进行拼接，最终只保存一个文件，其文件家夹名称将默认为
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
