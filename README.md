# ROS Extension for Scratch 3

![animation](https://user-images.githubusercontent.com/20625381/50626217-58147400-0f70-11e9-81ae-71d00fd18982.gif)

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

1. Access http://scratch3-ros.jsk.imi.i.u-tokyo.ac.jp

2. Open a terminal and fire up the following command
```
roslaunch rosbridge_server rosbridge_websocket.launch
```

3. Add the `ROS Extension` from the bottom left button and input `localhost` as the master URI

## Examples

Examples can be found at the [examples directory](https://github.com/Affonso-Gui/scratch3-ros-vm/tree/develop/src/extensions/scratch3_ros/examples). To run the examples:

1. On a terminal, launch `roslaunch rosbridge_server rosbridge_websocket.launch`
2. Access http://scratch3-ros.jsk.imi.i.u-tokyo.ac.jp and load the example file
3. Click on the warning sign near the ROS blocks menu to connect with rosbridge. Use `localhost` as the master URI.
![warning](https://user-images.githubusercontent.com/20625381/50582008-55e3e400-0ea2-11e9-942e-496bda7c557a.png)
4. Check comments for other required nodes
5. Click the green flag to start!

## Blocks API

Details of provided blocks can be found at [BLOCKS.md](https://github.com/Affonso-Gui/scratch3-ros-vm/blob/develop/src/extensions/scratch3_ros/BLOCKS.md).


## Run from Source

Git clone the repositories below and follow instructions at https://github.com/LLK/scratch-gui/wiki/Getting-Started
- https://github.com/Affonso-Gui/scratch3-ros-gui
- https://github.com/Affonso-Gui/scratch3-ros-vm
- https://github.com/Affonso-Gui/scratch3-ros-parser

## Develop a library for your own robot

You can include Scratch3-ROS on your Scratch project and create custom block libraries for your own robot. An example is given in the `fetch_extension` branch:
- https://github.com/Affonso-Gui/scratch3-ros-vm/tree/fetch_extension
- https://github.com/Affonso-Gui/scratch3-ros-gui/tree/fetch_extension
