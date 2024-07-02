# Creating Drivetrain Object

Here we will be making the robot drivetrain object:

```cpp
WhoopDrivetrain robot_drivetrain(
  &odom_fusion, // Odometry fusion module
  PoseUnits::in_deg_cw, // Set default pose units if not defined. "m_deg_cw" means "meters, degrees, clockwise-positive yaw", "in_deg_ccw" means "inches, degrees, counter-clockwise-positive yaw", and so forth.
  &controller1, // Pointer to the controller 
  &left_motors, // Pointer to the left motor group (optionally can be a list of motors as well)
  &right_motors // Pointer to the right motor group (optionally can be a list of motors as well)
);
```

There are different types of pose units you can use:

| ```PoseUnits```     | Definition | 
|----------|:--------:|
| ```in_deg_cw```    | Inches, Degrees, Clockwise-Positive     |
| ```in_deg_ccw```    | Inches, Degrees, Counter-Clockwise-Positive     |
| ```in_rad_cw```    | Inches, Radians, Clockwise-Positive     |
| ```in_rad_ccw```    | Inches, Radians, Counter-Clockwise-Positive     |
| ```m_deg_cw```    | Meters, Degrees, Clockwise-Positive     |
| ```m_deg_ccw```    | Meters, Degrees, Counter-Clockwise-Positive     |
| ```m_rad_cw```    | Meters, Radians, Clockwise-Positive     |
| ```m_rad_ccw```    | Meters, Radians, Counter-Clockwise-Positive     |

PoseUnits has several types that you can use as your standardized field system. Here is an image example of ```in_deg_cw``` explaining how the pose system on the field will work:

![Image](../images/OdomUnits.png)

As you see, ```(0,0)``` with a yaw of 0 displays the robot at position ```(0,0)``` facing parallel to the auton line.

If we move the robot to ```(24, 43)``` with a yaw of ```-130```, the position of the robot will be as follows:

![Image](../images/OdomUnitsExample.png)

You can see that the robot moved counter-clockwise. Since the drivetrain object is configured to be clockwise-positive, that is why the yaw is negative.

Now, you are ready to move on to [Creating Manager Object](CreatingManagerObject/README.md).