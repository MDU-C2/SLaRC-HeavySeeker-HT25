# Disclaimer
This package contains a robot description modular framework to create a URDF file to decribe the robot.

The files are of the .sdf.xacro extention, it enables to combine multiple files for specific parts in one assembly file.

The launch file contains an automatic conversion from .sdf.xacro --> .sdf which is a "Simulation Description File (SDF)" but functioning as a URDF beacuse it is compatable with the Robot State Publisher node.
 _Ignore the warnings during launch, it is fine_

# This covers 3 different topics: 
1. Edit Assembly
2. Adding a new sensor
3. Adding a new UGV
4. Simulation specific

> For more information on sdf visit https://sdformat.org/spec/

# 1) Edit Assembly

## CHANGE THE POSITION OF A COMPONENT

At the top of an assembly file is xacro properties, change the value to update position. Example:

```<xacro:property name="gnss_x_offset" value="${0*cm2m}"/>```




## ADD AN EXISTING SENSOR

To add a sensor that alredy exist use the xacro in the assembly.
The position will be changed in relative to the specified parent link when loading in the sensor. Here you also have the option to rotate the sensor. Example:

```<xacro:navsat_gnss parent_link="rig_top_link" xyz="${gnss_x_offset} ${gnss_y_offset} ${gnss_z_offset}" rpy="0 0 0"/>```

If adding more than one of that sensor make sure to specify individual names by setting the name param. Otherwise default name will be used. Example:


```<xacro:navsat_gnss parent_link="rig_top_link" name="gnss_1" xyz="${gnss_x_offset} ${gnss_y_offset} ${gnss_z_offset}" rpy="0 0 0"/>```



## CHANGE UGV
If changing UGV always set base_link = "UGV_link" since that is the canonical link.



# 2) Adding a new sensor
If creating a new xacro for a new sensor there are a few things to keep in mind. The file should be located and named accordingly: model/Sensors/name_of_sensor/model.sdf.xacro



## FILE TYPE
All components MUST be of type .sdf.xacro 

Most publicly avaliable resources if of type .sdf but it is farily easy to convert.
Just add the xacro functionallity in the format description tag. Example:

```
<sdf version="1.11" xmlns:xacro="http://ros.org/wiki/xacro">
   <xacro:macro name="name_of_sensor" params="">
    ..........Your Code Here.........
    </xacro:macro>
</sdf>
```
## PARAMS
Should take in the params name,parent_link, xyz & rpy. Exmaple:

```<xacro:macro name="oakd" params='name:=oakd_pro parent_link xyz:="0 0 0" rpy:="0 0 0"'>```




## LINK TO PARENT_LINK
All URDFs usally has a base_frame/base_link that all other links is connected to. You should set the pose of that link in relative to the parent_link. Example:

```<pose relative_to="${parent_link}">${xyz} ${rpy}</pose>```

>Avoid using joints for positioning as that has proven to be unpredictable

It is also possible to use the parent_link as the base_frame/base_link directly.



# 3) Adding a new UGV

If creating a new xacro for a new UGV there are a few things to keep in mind. The file should be located and named accordingly: model/UGV/<name_of_UGV>/model.sdf.xacro

## FILE TYPE
All components MUST be of type .sdf.xacro 

Most publicly avaliable resources if of type .sdf but it is farily easy to convert.
Just add the xacro functionallity in the format description tag. Example:
```
<sdf version="1.11" xmlns:xacro="http://ros.org/wiki/xacro">
    <xacro:macro name="name_of_UGV" params=''>
    ..........Your Code Here.........
    </xacro:macro>
</sdf>
```


## LINK TO PARENT_LINK
All URDFs usally has a base_frame/base_link that all other links is connected to. You should set the pose of that link in relative to the parent_link. Example:
```
<link name="base_link">
    <pose relative_to="${parent_link}">0 0 0 0 0 0</pose>
```
It is also possible to use the parent_link as the base_frame/base_link directly.





# 4) Simulation specific


## PLUGINS
Beware that some plugins used also need to be added in the world file. Example:

In world file:
```<plugin name="gz::sim::systems::Imu" filename="gz-sim-imu-system"/>```

This will enable the imu tag for example.