# Distributed system providing gesture-based interface

The system was distributed to multiple computational units Jetson Nano due to the high requirements for processing data from each camera. Python wheel of MediaPipe for Jetson Nano compiled with CUDA support can be found here: [Google's MediaPipe (v0.8.9) and Python Wheel installer for Jetson Nano (JetPack 4.6) compiled for CUDA 10.2](https://github.com/anion0278/mediapipe-jetson)

![image](https://user-images.githubusercontent.com/23017063/152826484-c1a98058-3cad-41eb-90f8-643fe03dba19.png)

## Related publications
*   [Hand Gesture Interface for Robot Path Definition in Collaborative Applications: Implementation and Comparative Study](https://doi.org/10.3390/s23094219)
```
@Article{s23094219,
AUTHOR = {Vysocký, Aleš and Poštulka, Tomáš and Chlebek, Jakub and Kot, Tomáš and Maslowski, Jan and Grushko, Stefan},
TITLE = {Hand Gesture Interface for Robot Path Definition in Collaborative Applications: Implementation and Comparative Study},
JOURNAL = {Sensors},
VOLUME = {23},
YEAR = {2023},
NUMBER = {9},
ARTICLE-NUMBER = {4219},
URL = {https://www.mdpi.com/1424-8220/23/9/4219},
ISSN = {1424-8220},
DOI = {10.3390/s23094219}
}
```

## Quick Start
All camera nodes and the master must be set to distibuted ROS graph mode.

Launching the dirstibuted system with three camera nodes:
```
roslaunch jetson_camera_node start_distributed_graph.launch
```
Start the aggregator after the camera nodes started:
```
rosrun jetson_camera_node main_node_processor.py
```
