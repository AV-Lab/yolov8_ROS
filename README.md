# yolov8_ROS
Detect agents with yolov8 in real-time and publish detection info via ROS 


## Required Packages:
Since this package is based on [ultralytics/yolov8], python>=3.8 is required.

You need to have ultralytics installed in your workspace. You can do it by running the following commands:   

```pip install ultralytics```

You will also need Rospy. You can install it by running the following commands:   

```sudo apt-get install python-rospy```

For visiualization purposes you might need opencv. You can install it by running the following commands:   

```pip install opencv-python```

## How to run:
1. Clone this repository in your workspace.
2. Install the packages needed.
3. Run the following command in your workspace:   

    ```python3 yolov8_ros.py```
