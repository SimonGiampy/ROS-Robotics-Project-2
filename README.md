
# Robotics Project 2 - Academic Year 2021/2022

<p align="center">
  <img width="500" src="images/polimi_logo.png" alt="PoliMi Logo" />
  <br>
</p>

## Contributors

- [__Marco D'Antini__](https://github.com/DantiniMarco) (10603556)
- [__Simone Giampà__](https://github.com/SimonGiampy) (10659184)


## Project structure

|Directory|File|Description|
|---------------|---------|-----------|
| /bags |  `robotics1_final.bag` | bag used for gmapping
|  |  `robotics2_final.bag`| bag used for localization 
|  |  `robotics3_final.bag`| bag used for localization 
| /cfg | `amcl.xml` | configuration file for amcl localization 
|  | `gmapping.xml` | configuration file for gmapping 
| /launch | `amcl.launch` | launch file for amcl 
|  | `gmapping.launch` | launch file for gmapping 
|  | `scan_merger.launch` | launch file used to merge the two laser scans, called by both amcl and gmapping
| /maps | `map1.pgm` | the map built with gmapping 
|  | `map1.yaml` | metadata of the map 
| /rviz | `amcl_cfg.rviz` | rviz configuration file 
|  | `map_cfg.rviz` | rviz configuration file 
| /src | `odomt_tf2.cpp` | node that subscribes to the `/odom` topic and transform it with __child_frame_id__ `base_link`
| /srv | `TrajectorySrv.srv` | empty service message
| /script | `trajectory_map.py` | script implementing the service that prints the image with map and trajectory 

## How to start gmapping

In order to start all the nodes type the following command:

```
roslaunch project2 gmapping.launch bag_file:=robotics1_final
```

this is going to launch the following nodes: 
- __gmapping__ 
- __scan merger__ 
- the __bag__ player (the bag file name can be modified with the related parameter via terminal)
- the cpp file __odom_tf2__
- __rviz__ with its configuration
## How to start amcl 

In order to start all the nodes type the following command:

```
roslaunch project2 amcl.launch bag_file:=robotics1_final
```

this is going to lauch the following nodes: 
- __amcl__ 
- __scan_merger__
- the __bag__ (the bag file name can be modified with the related parameter via terminal)
- the cpp file __odom_tf2__
- __rviz__ with its configuration

## Map image Service
We used this command to obtain the .pgm file of the __map__ once gmapping algorithm ended.
```
rosrun map_server map_saver -f map1
```
## Map with trajectory Service
We decided to write down a __Python script__ to deal with the request of printing the image of the map and trajectory via service. 

The script works like following: 
- subscribes to the `/map` topic of Rviz and gets the image of the map
- subscribes to the `/amcl_pose` topic of Rviz and gets the __x__ and __y__ coordinates of the amcl algorithm while processing
- for each of these position draws a symbol (__+__) on the image of the map. The image was not initially centered so we calculated some __scaling and traslation__ adjustments to make the position more accurate.

While amcl is running, run the Python script:
```
rosrun project2 trajectory_map.py
```
then call the service:
```
rosservice call trajectory
```

to print the map with the trajectory. The image is saved in the __/maps folder__, titled "_map_trajectory_" 
## Structure of the TF tree

We visualized the structure of the TF tree thanks to the following command:

```
rosrun rqt_tf_tree rqt_tf_tree
```

Thus, we obtained the following:

<p align="center">
  <img src="images/tf_tree.png" alt="TF Tree" />
  <br>
</p>

The following is the ros graph obtained with rqt graph tool :

<p align="center">
  <img src="images/rosgraph.png" alt="rosgraph" />
  <br>
</p>

## Map 

This is the obtained map: 
<p align="center">
  <img src="map/map1.pgm" alt="map1" />
  <br>
</p>






