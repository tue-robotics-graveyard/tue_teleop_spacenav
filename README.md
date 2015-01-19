# TU/e teleop spacenav

## Installation
Clone the repository in your catkin workspace and catkin_make.
Then install all dependencies (spacenavd):
```
$ rosdep install tue_teleop_spacena
```

## Running
Check if `spacenavd` is running
```
$ service spacenavd status
```
If it is not running, execute the following command:
```
service spacenavd start
```

After that, launch the appropriate launch file.
