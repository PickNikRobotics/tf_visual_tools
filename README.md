# TF Visual Tools

Easily move `\tf` frames around using your keyboard or mouse. Use as a simple calibration-by-eye tool or add menus and interative markers to create customized functionality.

![](resources/demo_screenshot.png)
Screenshot of the TF Visual Tools GUI.

<img src="https://picknik.ai/assets/images/logo.jpg" width="120">

This open source project was developed at [PickNik Robotics](https://picknik.ai/). Need professional ROS development and consulting? Contact us at projects@picknik.ai for a free consultation.

## Install

### Build from Source

To build this package, ``git clone`` this repo into a [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) and be sure to install necessary dependencies by running the following command in the root of your catkin workspace:

    rosdep install -y --from-paths src --ignore-src --rosdistro kinetic
## Usage of TF Visual Tools:

* Start Rviz and add the TF Visual Tools GUI to Rviz: `Panels > Add New Panel`
* Then select the TF Visual Tools GUI: `tf_visual_tools/TFVisualTools`
* Start the TF publisher: `rosrun tf_visual_tools tf_visual_tools`

These steps are already completed when running `roslaunch tf_visual_tools demo.launch`

### Creating and Removing TFs
![](resources/add-remove.png)

#### To create a TF:
1. select a TF from the `from:` dropdown list or type a new name.
1. enter a name for the TF in the `to:` line.
1. check whether you would like the TF to have an [interactive marker](http://wiki.ros.org/rviz/Tutorials/Interactive%20Markers%3A%20Getting%20Started) associated with it.
1. check whether you would like to attach menu options to your interactive markers. If you do want menus, you will be prompted to select a text file that contains the menu titles.
1. click `Create TF`.

**Note:** see the `File Formats` section for details about file format.

**Note:** see the `Interactive Markers and Menu Selections` for details about using the menu items.

#### To remove a TF:
1. select the TF from the dropdown list.
1. click `Remove TF`

### Interactive Markers and Menu Selections

Due to limitations in the Interactive Markers package, all interactive markers will have menus if enabled and all menus will have the same choices.

For each menu item, a publisher will be created with the topic name:

For example: 

* Items without sub menus: `/imarker/menu_item_B`
* Items with sub menus: `/imarker/menu_item_A_sub_entry_1`

When the menu items are selected, a value of `true` will be published to the corresponding topic. Create a subscriber in your own code to handle menu selections.

### Manipulating TFs
![](resources/manipulate.png)

The TFs you create can be manipulated through the GUI or the keyboard.

#### Manipulation with the GUI:
1. select the TF that you would like to manipulate from the dropdown list.
1. set the individual degree of freedom values or use the `+` and `-` buttons to increment the desired degree of freedom. The incrementation is determined by values set in `Delta xyz` and `Delta rpy`.

#### Manipulation with the Keyboard:
1. Select the TF that you would like to manipulate from the dropdown list.
1. Use the following commands to manipuate the TF:

```
Move: X Y Z R P Yaw
-------------------
+     q w e r t y
-     a s d f g h
fast  u
med   i
slow  o
```

**Note:** The `Manipulate` tab must have focus. If the keyboard commands are not working, try clicking the tab again as it probably lost focus.

#### Manipulation with the Interactive Markers:
1. Interactive Marker positions are changed by dragging either the grey box or grabbing one of the arrows or rings to change position or rotation of a single degree of freedom.

### Saving and Loading TFs
![](resources/save-load.png)

These buttons will either save or load a group of TFs.

## File Formats

There are two types of files used in the `tf_visual_tools` package:

* saved TFs
* interactive marker menus

### Saved TF file format

```
#ID From To IMarker Menus X Y Z ROLL PITCH YAW
# (comment line)
# space delimeter
# very little error checking done... so check your input.
0 map a 1 1 0.00 0.00 0.00 0.00 0.00 0.00
1 a b 1 1 0.50 1.50 0.50 0.00 0.00 0.00
... and so on ...
```

`ID` - an integer id for the frame.

`From` and `To` - strings with the corresponding TF names.

`IMarker` and `Menus` - bool (0 or 1) indicating whether the TF has an interactive marker and menus.

`X`, `Y`, `Z`, `ROLL`, `PITCH`, `YAW` - doubles corresponding to the TFs degrees of freedom.

### Interactive Marker Menus

```
# example menu file
# (shebang anywhere in line comments line)
# FORMAT: (supports only 1 sub level, sub menus cannot have sub menus) 
# (num sub menus), menu title 
# submenu title
# very little error checking done... so check your input.
3, menu item A
sub entry 1
sub entry 2
sub entry 3
0, menu item B
0, menu item C
2, menu item D
sub entry 1
sub entry 2
```
