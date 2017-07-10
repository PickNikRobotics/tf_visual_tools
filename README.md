# TF Visual Tools

Easily move `\tf` frames around using your keyboard or mouse. Use as a simple calibration-by-eye tool or add menus and interative markers to create customized functionality.

TF Visual Tools developed by [Andy McEvoy](http://github.com/mcevoyandy) and [Dave Coleman](http://dav.ee).

Status:
* TODO: BUILD STATUS

![](resources/demo_screenshot.png)
Screenshot of the TF Visual Tools GUI.

## Install

### Ubuntu Debian

`sudo apt-get install ros-kinetic-tf-visual-tools`

### Build from Source

To build this package, ``git clone`` this repo into a [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) and be sure to install necessary dependencies by running the following command in the root of your catkin workspace:

    rosdep install -y --from-paths src --ignore-src --rosdistro indigo

## Code API

* TODO: CLASS REFERENCE

## Usage of TF Visual Tools:

* Start Rviz and add the TF Visual Tools GUI to Rviz: `Panels > Add New Panel`
* Then select the TF Visual Tools GUI: `tf_visual_tools/TFVisualTools`
* Start the TF publisher: `rosrun tf_visual_tools tf_visual_tools`

### Creating and Removing TFs
![](resources/add-remove.png)


### Manipulating TFs
![](resources/manipulate.png)

### Saving and Loading TFs
![](resources/save-load.png)
