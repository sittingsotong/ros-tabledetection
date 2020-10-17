# ros-tabledetection

## What it does 
A ROS package that creates a ROS node called ```detect``` that reads data in the form of a ```sensor_msgs/Image``` message stored in a ```.bag``` file. Using OpenCV, it performs object detection such that the front legs of the rack is detected. It then publishes the detected results as a ```sensor_msgs/Image``` ROS message to the topic ```/image_topic```. 

## How to run 
### Dependencies 
1. ROS Noetic
2. Catkin
3. opencv-python
4. numpy
5. rosbag
6. cv_bridge 

### Set up
1. Clone the package into your catkin workspace
``` 
cd [your_workspace/src]
git clone https://github.com/sittingsotong/ros-tabledetection
cd .. 
catkin_make
source /devel/setup.bash (for bash users)
source /devel/setup.zsh (for zsh users)
```

2. Place ```.bag``` file in the directory ```data/rack_test```, or any other name you prefer. Once this is done, open the ```detect.launch``` file and edit the ```value``` field under the ```<param>``` tag to the correct path and filename

```
<param name="file_path" value ="$(find rack)/data/rack_test/2019-02-27-19-35-19.bag" />
```

3. While in the main directory, run the following command:
```
chmod u+x bin/detect
```

### Running 
Once all the setting up is complete, run the following command 
``` 
roslaunch rack detect.launch
```

Two windows will pop up, one showing the detected table legs, and the other showing the original unedited photo. To view the next frame, press the button ```0``` on your keyboard. Once the key is pressed, the image will be converted and published to the ROS topic ```image_topic```.

## Reasons behind approach taken
Given 2 different types of ROS messages: ```sensor_msgs/Image``` and ```sensor_msgs/LaserScan```, I chose to work with the former as image processing using OpenCV would be easier. Especially with the ```cv_bridge``` package, conversion between the ROS message and an OpenCV image was made simple, and thus the hassle of trying to decode the ROS message would be eliminated. 

The steps taken to detect the table edge is in the order below:
1. Filtering the image by colour
2. Applying blurring and smoothing to remove unwanted noise
3. Extracting vertical lines
4. Getting edges from the vertical lines
5. Final smoothing of the results

Filtering the image by colour was a simple process that easily helped to single out the rack from everything else in the surrounding. In addition, slightly adjusting the upper and lower bound of the filtering also caused the back legs of the table to be filtered out (because of the shadows), which made everything else much simpler. Of course, this method may not work as effectively if the angle of lights or camera angle were to be adjusted. 

In order to only see the legs of the table, the horizontal platform of the rack has to be eliminated. Hence, I decided to use OpenCV's morphology method to filter out only the vertical lines present. Adjusting the kernel parameters of this step also helped to clean up the result, although there is still some noise from the horizontal platform. 

### Intermediary Steps taken
In order to get the topic of the message in the rosbag file, the following command was ran:
```
rosbag info data/rack_test/2019-02-27-19-35-19.bag
```
and the following result was obtained:
```
path:        data/rack_test/2019-02-27-19-35-19.bag 
version:     2.0 
duration:    1:50s (110s) 
size:        8.5 GB 
messages:    4965 
compression: none [3310/3310 chunks] 
types:       sensor_msgs/Image     [060021388200f6f0f447d0fcd9c64743] sensor_msgs/LaserScan [90c7ef2dc6895d81024acba2ac42f369] 
topics:      
mono_front/usb_cam/image_rect_color   3310 msgs    : sensor_msgs/Image 
scan                                  1655 msgs    : sensor_msgs/LaserScan
```

### Afterthoughts
While this solution is able to accurately detect the front legs of the rack, it is dependent on many variables: lighting, camera angle, blockages. Hence, for a reliable long-term solution, this may not the best method.

A possible improvement could possibly require more data that uses these different camera angles or lighting. Then using these data to create a machine learning algorithm that performs object detection on the data provided. 

### Video demonstration
