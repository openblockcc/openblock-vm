# ROS Extension for Scratch 3

An extension to connect [Scratch 3](https://en.scratch-wiki.info/wiki/Scratch_3.0) to [ROS](http://wiki.ros.org/) enabled platforms!

Supports:
- publishing and subscribing topics
- calling services
- getting and setting rosparam variables

## About

This extension packs with utility blocks for creating and manipulating [JSON objects](https://www.w3schools.com/js/js_json_objects.asp), which are integrated to Scratch variables and used to represent ROS messages.

When communicating with the ROS interface, message types are mostly infered by the topic or service name, being occluded from the user. This way the user do not need to worry that much about types, allowing easier and more intuitive usage of this extension.

This also means, however, that this extension doesn't do well in advertising new topics or serving services. Maybe these will be supported in future releases, but for now Scratch interface is designed to act as a ROS **client**, publishing to topics and called nodes already advertised by some other node, which should be responsible to handle the message from Scratch and do all of the robotics stuff.

## Quick Start
0. [Install ROS](http://wiki.ros.org/ROS/Installation) and the following dependencies. This project was tested on ROS kinetic, but should run well in other distributions as well.
```bash
# Install main dependencies
sudo apt install ros-kinetic-rosbridge-server
# Install examples dependencies
sudo apt install ros-kinetic-turtlesim ros-kinetic-actionlib-tutorials 
```

1. Access https://affonso-gui.github.io/scratch-gui

2. Open a terminal and fire up the following command
```
roslaunch rosbridge_server rosbridge_websocket.launch
```

3. Add the `ROS Extension` from the bottom left button

## Examples

Examples can be found at the `examples` directory. To run the examples:

1. On a terminal, launch `roslaunch rosbridge_server rosbridge_websocket.launch`
2. Access https://affonso-gui.github.io/scratch-gui/ and load the downloaded example
3. Click on the warning sign near the ROS blocks menu to connect with rosbridge
![warning](https://user-images.githubusercontent.com/20625381/50582008-55e3e400-0ea2-11e9-942e-496bda7c557a.png)
4. Check comments for other required nodes
5. Click the green flag to start!

## Blocks API

Details of provided blocks can be found at [BLOCKS.md](https://github.com/Affonso-Gui/scratch-vm/blob/develop/src/extensions/scratch3_ros/BLOCKS.md).


## Run from Source

Git clone the repositories below and follow instructions at https://github.com/LLK/scratch-gui/wiki/Getting-Started
- https://github.com/Affonso-Gui/scratch-gui
- https://github.com/Affonso-Gui/scratch-vm
- https://github.com/Affonso-Gui/scratch-parser