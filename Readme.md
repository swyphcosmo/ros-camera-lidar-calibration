# Camera and LIDAR Calibration and Visualization in ROS

# Setting up ROS in vagrant

```shell
$ vagrant init shadowrobot/ros-indigo-desktop-trusty64
```

* Open `Vagrantfile` in editor
	* uncomment/edit these lines:

```shell
  config.vm.provider "virtualbox" do |vb|
  #   # Display the VirtualBox GUI when booting the machine
    vb.gui = true
  #
  #   # Customize the amount of memory on the VM:
    vb.memory = "2048"
  end
```

```shell
$ vagrant up
```

* Wait for box to download and provision

* When gui boots, open Terminal

```shell
$ sudo apt-get update
$ sudo apt-get upgrade
```

# Inspecting bag file

* Move '.bag' file to the same folder as Vagrantfile

Inside VM:

```shell
$ rosbag info 2016-11-22-14-32-13_test.bag
``` 

Sample output:

```shell
path:        2016-11-22-14-32-13_test.bag
version:     2.0
duration:    1:53s (113s)
start:       Nov 22 2016 16:32:14.41 (1479853934.41)
end:         Nov 22 2016 16:34:07.88 (1479854047.88)
size:        3.1 GB
messages:    5975
compression: none [1233/1233 chunks]
types:       sensor_msgs/CameraInfo  [c9a58c1b0b154e0e6da7578cb991d214]
             sensor_msgs/Image       [060021388200f6f0f447d0fcd9c64743]
             sensor_msgs/PointCloud2 [1158d486dd51d683ce2f1be655c3c181]
topics:      /sensors/camera/camera_info   2500 msgs    : sensor_msgs/CameraInfo 
             /sensors/camera/image_color   1206 msgs    : sensor_msgs/Image      
             /sensors/velodyne_points      2269 msgs    : sensor_msgs/PointCloud2
```

# Playing bag file

Inside VM:

```shell
$ rosbag play 2016-11-22-14-32-13_test.bag
```

To play at slower speed, e.g. 50%:

```shell
$ rosbag play -r 0.5 2016-11-22-14-32-13_test.bag
```

**Note:** This may be required depending on the speed of the host PC.

# Task #1: Camera Calibration

**Note:** The checker board pattern used 5 x 7 corners and size of each square 5 cm.

## Automatic Calibration using `cameracalibrator.py`

Following tutorial from [ROS Wiki](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration)

```shell
$ rosdep install camera_calibration
$ rosmake camera_calibration
```

Inside VM:

```shell
$ rosrun camera_calibration cameracalibrator.py --size=5x7 --square=0.050 image:=/sensors/camera/image_color camera:=/sensors/camera/camera_info  --no-service-check
```

* Bag file was played back at 50% speed to allow `cameracalibrator.py` to collect enough images to cover the X, Y, Size, and Skew parameter spaces. 
* 24 images were collected

### Results

```yaml
image_width: 964
image_height: 724
camera_name: narrow_stereo
camera_matrix:
  rows: 3
  cols: 3
  data: [483.306502, 0.000000, 456.712456, 0.000000, 482.958638, 366.254245, 0.000000, 0.000000, 1.000000]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [-0.197847, 0.065563, 0.003166, -0.000043, 0.000000]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1.000000, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000, 0.000000, 0.000000, 1.000000]
projection_matrix:
  rows: 3
  cols: 4
  data: [409.833832, 0.000000, 456.584871, 0.000000, 0.000000, 410.319702, 370.492937, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000] 
```

## Manual Calibration

* Use `image_view` to collect images. Right click to save screenshot.

```shell
$ rosrun image_view image_view image:=/sensors/camera/image_color
```

* 30 images were saved as `frame0000.jpg` to `frame0029.jpg`.
* `part1.py` was created to perform calibration using these images.

```python
import cv2
from camera_calibration.calibrator import MonoCalibrator, ChessboardInfo

numImages = 30

images = [ cv2.imread( '../Images/frame{:04d}.jpg'.format( i ) ) for i in range( numImages ) ]

board = ChessboardInfo()
board.n_cols = 7
board.n_rows = 5
board.dim = 0.050

mc = MonoCalibrator( [ board ], cv2.CALIB_FIX_K3 )
mc.cal( images )
print( mc.as_message() )

mc.do_save()
```

Run `part1.py`

```shell
$ cd scripts
$ python part1.py
```

### Results

```yaml
image_width: 964
image_height: 724
camera_name: narrow_stereo/left
camera_matrix:
  rows: 3
  cols: 3
  data: [485.763466, 0.000000, 457.009020, 0.000000, 485.242603, 369.066006, 0.000000, 0.000000, 1.000000]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [-0.196038, 0.062400, 0.002179, 0.000358, 0.000000]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1.000000, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000, 0.000000, 0.000000, 1.000000]
projection_matrix:
  rows: 3
  cols: 4
  data: [419.118439, 0.000000, 460.511129, 0.000000, 0.000000, 432.627686, 372.659509, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000] 
```

## Rectifying images

### Adding calibration information to bag files

* Install `bag_tools`

```shell
$ sudo apt-get install ros-indigo-bag-tools
```

**Note:** `rosrun` couldn't find `change_camera_info.py`, but it was in `/opt/ros/indigo/lib/python2.7/dist-packages/bag_tools` so it was copied into the `scripts` directory.

* Create new bagfiles with calibration data

For `cameracalibrator.py` output:

```shell
python change_camera_info.py ../2016-11-22-14-32-13_test.orig.bag ../2016-11-22-14-32-13_test.cameracalibrator.bag /sensors/camera/camera_info=../Results/calibrationdata_cameracalibrator.yaml
```

For `part1.py` output:

```shell
python change_camera_info.py ../2016-11-22-14-32-13_test.orig.bag ../2016-11-22-14-32-13_test.part1.bag /sensors/camera/camera_info=../Results/calibrationdata_part1.yaml
```

### Create launch file for recording rectified image

`image_proc` was used to rectify the image based on the new calibration information. 

Three `.launch` files were created to record three different results: original image, rectified image using manual calibration data, and rectified image using automatic calibration data. 

The files for the rectified images were similar with the only difference being which bagfile was passed to `rosbag`. The `image_proc` node was removed when recording the original image. 

```xml
<launch>
	<node name="rosbag" pkg="rosbag" type="play" args="/vagrant/2016-11-22-14-32-13_test.cameracalibrator.bag"/>
	<node name="image_proc" pkg="image_proc" type="image_proc" respawn="false" ns="/sensors/camera">
		<remap from="image_raw" to="image_color"/>
	</node>
	<node name="rect_video_recorder" pkg="image_view" type="video_recorder" respawn="false">
		<remap from="image" to="/sensors/camera/image_rect_color"/>
	</node>
</launch>
```

By default, `video_recorder` creates `output.avi` in `/home/ros/.ros`. After running each launch file, the resulting `output.avi` was renamed and copied to the `../Results` directory.

### Compare Calibration Results

The three videos were placed side by side using `ffmpeg` in order to more easily compare the results of the image rectification.

```shell
$ ffmpeg -i calibration-original.avi -i calibration-part1.avi -i calibration-cameracalibrator.avi -filter_complex '[0:v]pad=iw*3:ih[int];[int][1:v]overlay=W/3:0[int2];[int2][2:v]overlay=2*W/3:0,drawtext=fontsize=60:fontcolor=#095C8D:fontfile=/usr/share/fonts/truetype/freefont/FreeSans.ttf:text='Original':x=W/6+100:y=25,drawtext=fontsize=60:fontcolor=#095C8D:fontfile=/usr/share/fonts/truetype/freefont/FreeSans.ttf:text='Manual':x=3*W/6+100:y=25,drawtext=fontsize=60:fontcolor=#095C8D:fontfile=/usr/share/fonts/truetype/freefont/FreeSans.ttf:text='Automated':x=5*W/6+100:y=25[vid]' -map [vid] -c:v libx264 -crf 23 -preset veryfast part1-combined.mp4
```

The resulting video can be found [here](https://www.youtube.com/watch?v=d5EWqrG8jNw).

[![IMAGE ALT TEXT HERE](http://img.youtube.com/vi/d5EWqrG8jNw/0.jpg)](http://www.youtube.com/watch?v=d5EWqrG8jNw)

**Note:** The videos are not synchronized, but they're close enough to see the results of the comparison.

The image below shows a representative capture of the calibrations. The original image is on the left, and the rectified images from the manual and automatic calibrations are in the middle and right, respectively. As you can see, the manual calibration does not correct the radial distortion at the far edges of the image; however, both calibrations show a rectified checker board in the center of the image. In typical use cases, both calibrations should be adequate. 

![Image Calibration Comparison](Results/Images/part1-video-screenshot.png)

