# FTC Robotics

This repository contains various projects I developed during high school based on the base FTC (FIRST Tech Challenge) repository for the 2023-2024 season, CENTERSTAGE. Aside from a basic mecanum drive and implementation of teleop controls, the code includes implementations of computer vision, neural networks for detecting autonomous elements, and autonomous PID controls for navigation. 

### 1. Computer Vision
- Utilizes OpenCV for object detection and tracking using AprilTags.
- Implements vision processing to assist with autonomous navigation.

### 2. Neural Networks
- Developed neural networks to detect and classify location of autonomous elements.
- Trained models to improve the accuracy and efficiency of autonomous operations.

### 3. PID Controls with RoadRunner
- Implemented PID control algorithms for precise robot movement.
- Integrated RoadRunner for trajectory planning and execution.
- Integrated MeepMeep for path visualization.

More specifically, an example of autonomous code using implementation of all three can be found at [TeamCode\src\main\java\org\firstinspires\ftc\teamcode\autos\RedSideAutoCycle.java](TeamCode\src\main\java\org\firstinspires\ftc\teamcode\autos\RedSideAutoCycle.java).

## Hardware Used

- **Control Hub**: The main control unit for the robot.
- **Motors**: GoBilda Yellow Jacket motors for driving and manipulating robot components.
- **Servos**: REV Robotics Smart Servos and GoBilda torque servos used for precise control of robot mechanisms.
- **Sensors**: Internal IMU, color sensors, and distance sensors for feedback and navigation.
- **Camera**: Used for computer vision tasks.

## Getting Started

To get started with these projects, clone the repository and follow the instructions in each project's directory.

## License

This project is copyrighted by FIRST - see the [LICENSE](LICENSE) file for details.

## Contributing

Feel free to contribute to this repository by submitting pull requests, and if you have any questions feel free to contact me at [aranjan@seas.upenn.edu](mailto:aranjan@seas.upenn.edu).

## Acknowledgments

- FTC (FIRST Tech Challenge) for providing the base repository.
- OpenCV, Roadrunner, MeepMeep and other open-source libraries used in these projects.
