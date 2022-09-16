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

```bash

wadda pcd . # view pcd file or pcd folder
```

### data collection

```bash
wadda dc # By filtering the default ros message type, and then logging the data
```

### gif generator

```bash
wadda gif . # generator gif from specify path
```

### voc2coco

```bash
wadda v2c . # convert voc to coco
```

### ros visualizer

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