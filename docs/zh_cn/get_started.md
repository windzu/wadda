# WADDA

> Wind's Autonomous Driving Development Art

This repository provides some handy tools and useful libraries

| Tools           | Description                                                          |
| --------------- |:--------------------------------------------------------------------:|
| pcd visualizer  | view point cloud files                                               |
| gif generator   | generate gif from image or point cloud                               |
| data collection | record data via ros                                                  |
| voc2coco        | convert standard voc format dataset to coco format                   |
| ros visualizer  | convert the common message format of ros to marker for visualization |

| Libs  | Description                             |
| ----- |:---------------------------------------:|
| pypcd | libraries for working with point clouds |
|       |                                         |

## Install

```bash
pip3 install wadda
```

The following is a brief introduction to its simple usage. For more advanced usage, please refer to its [documentation](https://wadda.readthedocs.io/en/latest/).

## Simple Usage

```bash
wadda [function_name] [path]
```

### pcd visualizer

* path : pcd file or the folder containing the pcd file

* tricks：If viewing a folder containing pcd files, use the "space" and "z" keys to control viewing the next and previous frames, and the "q" key to exit

```bash
wadda pcd . # view pcd file or pcd folder
```

### data collection

* path：Specify the path where you want the data to be stored

* pro：For advanced usage, please refer to the doc

```bash
wadda dc . # By filtering the default ros message type, and then store the data
```

### gif generator

> All folders under this path will be traversed. If a folder contains images or point cloud files, a gif will be generated with the name of the folder and stored in its parent directory

* path：Want to traverse the root path of the generated gif

```bash
wadda gif . # generator gif from specify path
```

### voc2coco

* path：The path where the standard voc format data set is located
  
  ```bash
  path
  ├── Annotations
  ├── coco
  ├── ImageSets
  ├── JPEGImages
  └── labels.txt
  ```

* labels.txt ：The labels.txt file must be included, and its content is the name of the category, which is used to map from class name to label id when converting to coco
  
  ```txt
  class_name_0
  class_name_1
  class_name_2
  ...
  ```

```bash
wadda v2c . # convert voc to coco
```

### ros visualizer

* pro：For advanced usage, please refer to the doc

```bash
wadda ros # start ros visualizer for converting ros msg
```

### pypcd

```python
from wadda import pypcd
# parse ros pointcloud2 data
pc = pypcd.PointCloud.from_msg(data)
x = pc.pc_data['x']
y = pc.pc_data['y']
z = pc.pc_data['z']

# parse pcd format file
pc = pypcd.PointCloud.from_path('foo.pcd')
x = pc.pc_data['x']
y = pc.pc_data['y']
z = pc.pc_data['z']
```