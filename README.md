![WhoopLib Logo](/docs/images/WhoopLibWhite.png)

<!-- PROJECT SHIELDS -->
<!--
*** I'm using markdown "reference style" links for readability.
*** Reference links are enclosed in brackets [ ] instead of parentheses ( ).
*** See the bottom of this document for the declaration of the reference variables
*** for contributors-url, forks-url, etc. This is an optional, concise syntax you may use.
*** https://www.markdownguide.org/basic-syntax/#reference-style-links
-->

The most advanced SLAM solution in VEX.

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![LinkedIn][linkedin-shield]][linkedin-url]

## Links

Documentation: https://connoratmos.github.io/WhoopLibVEXCode/#/

WhoopLibVEXCode Github: https://github.com/ConnorAtmos/WhoopLibVEXCode

WhoopLibPython Github: https://github.com/ConnorAtmos/WhoopLibPython

## Features

- Visual Odometry/Pose Estimation
- Wheel Odometry/Pose Estimation, inspired by JAR-Template
- Communication between V5 Brain and Jetson Nano
- Fusion Odometry between Visual Odometry and Wheel Odometry
- Rolling Average Filter for Fusion Odometry
- WhoopController Class with auto-configuration for Split Arcade, Tank, Left Stick Arcade, and Right Stick Arcade
- Dubins-Curves Path Creation, thanks to Andrew Walker
- Pure Pursuit Algorithm
- Moving between Point A and Point B
- Rolling Average, Kalman, and Low-Pass Filters
- General PID, modified from JAR-Template
- Turning by degrees, Turning to degrees, Turning to Face x and y Coordinates
- Moving Forward and Reverse Functions, alongside Remembering Previous Movement Positions for Improved Resilience
- Generated Paths with Waypoints to Navigate around the VEX Robotics Field in One Fell Swoop
- Well-Rounded and Continuously Updated Documentation for a Low Floor yet High Ceiling

## Roadmap

- Object Detection and Gridded Permanence system
- Detecting other robots that impede the path of the robot, and drive around
- Implementation of a Jetson Orion Nano instead of the End-Of-Line (EOL) Jetson Nano
- Implementation of a better SLAM solution instead of relying on the EOL Realsense T265 Camera
- Capability to Use Different Devices like Oak-D, etc. for Visual Odometry
- Capability to Use Lidar
- Virtual Highway system

## Downloading WhoopLibVEXCode Template for V5 Brain

On your computer (separate device) Install VSCode

Open up command prompt or terminal (On your computer):

```bash
  cd Desktop

  git clone https://github.com/ConnorAtmos/WhoopLibVEXCode
```

Open the project in VSCode

Install "VEX Robotics" Extension in VSCode

You should be prompted to install the C++ extension by Microsoft. This would be using Intellisense (clangd may not work with the VEXCode extension)

## Acknowledgements

 - [E-Bots πLons](http://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf): Odometry Documentation
 - [JAR-Template](https://github.com/JacksonAreaRobotics/JAR-Template): Odometry Inspiration
 - [Librealsense](https://github.com/IntelRealSense/librealsense): Depth Capturing
 - [VEX Robotics](https://github.com/VEX-Robotics-AI)
 - [Andrew Walker](https://github.com/AndrewWalker/Dubins-Curves/tree/master): Path Generation with Dubins-Curves

<!-- LICENSE -->
## License

Distributed under the [MIT](https://choosealicense.com/licenses/mit/) License.

<!-- CONTACT -->
## Contact

Connor White - connor.sw.personal@gmail.com

<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/ConnorAtmos/WhoopLibVEXCode.svg?style=for-the-badge
[contributors-url]: https://github.com/ConnorAtmos/WhoopLibVEXCode/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/ConnorAtmos/WhoopLibVEXCode.svg?style=for-the-badge
[forks-url]: https://github.com/ConnorAtmos/WhoopLibVEXCode/network/members
[stars-shield]: https://img.shields.io/github/stars/ConnorAtmos/WhoopLibVEXCode.svg?style=for-the-badge
[stars-url]: https://github.com/ConnorAtmos/WhoopLibVEXCode/stargazers
[issues-shield]: https://img.shields.io/github/issues/ConnorAtmos/WhoopLibVEXCode.svg?style=for-the-badge
[issues-url]: https://github.com/ConnorAtmos/WhoopLibVEXCode/issues
[license-shield]: https://img.shields.io/github/license/ConnorAtmos/WhoopLibVEXCode.svg?style=for-the-badge
[license-url]: https://github.com/ConnorAtmos/WhoopLibVEXCode/blob/master/LICENSE
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=for-the-badge&logo=linkedin&colorB=555
[linkedin-url]: https://www.linkedin.com/in/connor-white-38a5501a0/

