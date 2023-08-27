<!--
 * @Author: wind windzu1@gmail.com
 * @Date: 2023-08-27 18:30:27
 * @LastEditors: wind windzu1@gmail.com
 * @LastEditTime: 2023-08-27 18:46:58
 * @Description: 
 * Copyright (c) 2023 by windzu, All Rights Reserved. 
-->
# WADDA

> Wind's Autonomous Driving Development Art

This repository provides some handy tools and useful libraries

| Tools           | Description                                                          |
| --------------- |:--------------------------------------------------------------------:|
| gif generator   | generate gif from image or point cloud                               |
| data slice | slice data to nuscenes format                                                 |
| voc2coco        | convert standard voc format dataset to coco format                   |
| ros visualizer  | convert the common message format of ros to marker for visualization |



## Install

```bash
pip3 install wadda
```

The following is a brief introduction to its simple usage. For more advanced usage, please refer to its [documentation](https://wadda.readthedocs.io/en/latest/).

## Simple Usage

```bash
wadda [function_name] [path]
```



### data slice

* path：Specify the config file path


```bash
wadda ds ./config.yaml
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

## Development

```bash
git clone https://github.com/windzu/wadda.git
cd wadda
pip install -e .
```