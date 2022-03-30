<!--
# Bachelor_Thesis
Parametric mapping with a Jackal mobile robot
-->
<!-- #Info BA, time, Lübeck, Profs -->
This project is part of the bachelor thesis "**Traversability and Mapping of Obstacles in Uneven Terrain Based on a Robot's Design**".  
It was written by **Max-Ole von Waldow** at the **Universität zu Lübeck**.  
The thesis was supervised by **Dr. Ngoc Thinh Nguyen** and **Prof. Dr. Georg Schildbach**.

This tool analyses the traversability of rough terrain in regard to the design of a specific robot. Obstacles are mapped in a robot-centric map around the robot.

# Dependencies:
The project was developed for ROS Melodic.  
It requires the [elevation_mapping](https://github.com/ANYbotics/elevation_mapping) package and the [move_base](https://wiki.ros.org/move_base) package as an example for control software.  
Python 2.7 must be installed.

# Usage: 
Use the file `jackal_properties.yaml` as a template and input information about your mobile robot. 
To use a differently named file, adjust the filename in the `compile*.py` scripts.  

Use **conservative method:**  
This assumes worst control of the robot and can be used with any controller. By default move_base is launched. 
The resulting obstacle map might be restrictive if the robot can accelerate or drive too fast.  
1. Compile the filter chain by running `compile_robot_limitations.py`
2. Run `nav_in_traversabilityMap.launch`  

Use **non-conservative method:**  
This makes less conservative assumptions. The script `control_output_cap.py` limits the control output 
if necessary to keep the robot safe. By default move_base is launched.  
1. Compile with `compile_control_filterchain.py`
2. Run `nav_control_limited.launch`
	
# Thesis:
The thesis is available in the file `Bachelor_Thesis_Max-Ole_von_Waldow.pdf` and the slides for the presentation of the thesis are in the file `Colloquium_Powerpoint.pptx`.
## Abstract of the Thesis:
In unstructured environments, a robot’s stability depends on its design and
motion. In this thesis the traversability of rough terrain is determined and
tailored to the abilities of a mobile robot. We model dangerous situations that
could immobilize the robot or interfere with its desired path. These include
tip-over, sharp turns, the lack of motor power or getting stuck on edges. The
configuration of the robot is compiled to the generalized limit of the traversable
inclination and edge height. Obstacles are detected through these limits and
mapped inside a local region around a robot.
The presented process aims to be modular for easy adaptation. It is combined 
with control software that ranges from basic to more sophisticated. Simple 
controllers require conservative assumptions while more advanced software is
integrated more tightly and allows for less restrictive maps. We employ
the system in a field test at a skate park and illustrate the difference the type of
control software makes and the performance as a whole.
