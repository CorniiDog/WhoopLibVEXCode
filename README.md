
# Whooplib Template

The most advanced SLAM solution in VEX.


## Installation

[Instructions on setting up your Jetson Nano](https://docs.google.com/document/d/1Zwriuj1YhczBsMVh11xQKaoRo1WXXvaLQYEdnHrO4jg/edit?usp=sharing)

On your computer (separate device) Install VSCode

Install "VEX Robotics" Extension in VSCode

Installing Whooplib VEXCode (On your computer):

```bash
  cd Desktop

  git clone https://github.com/ConnorAtmos/WhoopLibVEXCode
```

Installing Whooplib Python (SSH On your Jetson Nano via "ssh jetson@your_jetson_ip"):

```bash
  cd ~/Desktop

  mkdir WhoopLibPython

  git clone https://github.com/ConnorAtmos/WhoopLibPython
```
    
## Documentation

The Documentation is planned to be developed. Will be updated upon creation.





## Acknowledgements

 - [Jar-Template](https://github.com/JacksonAreaRobotics/JAR-Template)
 - [Librealsense](https://github.com/IntelRealSense/librealsense)
 - [VEX Robotics](https://github.com/VEX-Robotics-AI)


## Features

- Visual Pose Estimation
- Communication between V5 Brain and Jetson Nano


## Roadmap

- Wheel Odometry
- Fusion between Visual Pose Estimation and Wheel Odometry
- Object Detection and Gridded Permanence systenm
- Moving between Point A and Point B
- Virtual Highway system
- Documentation
- Detecting other robots that impede the path of the robot, and drive around
- Virtual Highway system
## License

[MIT](https://choosealicense.com/licenses/mit/)

