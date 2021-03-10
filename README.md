## Requirements

- PCL 1.8
- OpenCV 3.4.2

## How to use

### 1. Make data folder

- Format

```
folder_name
├──xxx.png
├──xxx.pcd
├──yyy.png
├──yyy.pcd
├──...
```

You should specify thie image and point cloud data from the same frame for xxx.png and xxx.pcd.

PNG images are only supported.

### 2. Build this project

In this project,

```
$ mkdir build
$ cd build
$ cmake ..
$ make
```

### 3. Run

In build folder,

```
$ ./Calibrator <folder_path>
```
