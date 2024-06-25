![WhoopLib Logo](/include/whooplib/images/WhoopLibWhite.png)

<!-- PROJECT SHIELDS -->
<!--
*** I'm using markdown "reference style" links for readability.
*** Reference links are enclosed in brackets [ ] instead of parentheses ( ).
*** See the bottom of this document for the declaration of the reference variables
*** for contributors-url, forks-url, etc. This is an optional, concise syntax you may use.
*** https://www.markdownguide.org/basic-syntax/#reference-style-links
-->
[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![MIT License][license-shield]][license-url]
[![LinkedIn][linkedin-shield]][linkedin-url]

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
    
Modify the whooplibpython.service if necessary, then after:

```bash
  sudo cp whooplibpython.service /etc/systemd/system/whooplibpython.service

  sudo systemctl enable whooplibpython.service 

  sudo systemctl restart whooplibpython.service
```

## Documentation

The Documentation is planned to be developed. Will be updated upon creation.





## Acknowledgements

 - [E-Bots πLons](http://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf)
 - [Librealsense](https://github.com/IntelRealSense/librealsense)
 - [VEX Robotics](https://github.com/VEX-Robotics-AI)


## Features

- Visual Odometry/Pose Estimation
- Wheel Odometry/Pose Estimation
- Communication between V5 Brain and Jetson Nano


## Roadmap

- Fusion between Visual Pose Estimation and Wheel Odometry
- Object Detection and Gridded Permanence systenm
- Moving between Point A and Point B
- Documentation
- Detecting other robots that impede the path of the robot, and drive around
- Virtual Highway system

<!-- LICENSE -->
## License

Distributed under the [MIT](https://choosealicense.com/licenses/mit/) License.

<!-- CONTACT -->
## Contact

Connor White - connor.sw.personal@gmail.com

Project Link: [https://github.com/ConnorAtmos/WhoopLibVEXCode](https://github.com/ConnorAtmos/WhoopLibVEXCode)

<p align="right">(<a href="#readme-top">back to top</a>)</p>


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
[license-url]: https://github.com/ConnorAtmos/WhoopLibVEXCode/blob/master/LICENSE.txt
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=for-the-badge&logo=linkedin&colorB=555
[linkedin-url]: https://www.linkedin.com/in/connor-white-38a5501a0/

