
# MazeMaster: Autonomous maze-solving robot with cloud-based real-time navigation

![Rover](MazeMaster_Rover.jpg)

MazeMaster is an autonomous, two-wheeled robot designed to navigate a maze while mapping its layout in real-time. The robot utilizes an FPGA-based camera, accelerometer, gyroscope, compass, and wheel encoders to detect and avoid illuminated maze walls. It communicates with a cloud server to process navigation instructions and display a live map of the maze on a web application. This project combines robotics, embedded systems, cloud computing, and web development to deliver a fully functional autonomous navigation system.


## Features

- **Autonomous Navigation**: The robot uses a Depth-First Search (DFS) algorithm to navigate the maze, identifying junctions and avoiding retracing paths.
- **Real-Time Mapping**: A live map of the maze is displayed on a web application, updated in real-time as the robot moves.
- **FPGA-Based Vision System**: The robot detects colored beacons and white LED strips using an FPGA camera, enabling accurate localization and mapping.
- **Cloud Integration**: The robot communicates with a Python server hosted on AWS EC2, which processes data and stores maze node information in a DynamoDB database.
- **Web Application**: A React and Node.js web app visualizes the robot's movements and highlights the shortest path through the maze.
- **Self-Balancing**: The robot maintains balance on two wheels using a PID control system implemented on an Arduino Nano.



## Technologies Used

### Hardware
- **FPGA Camera**: For color detection and lane tracking.
- **ESP32 Microcontroller**: Handles communication with the server and controls the robot's movements.
- **Arduino Nano**: Implements the PID control system for balancing.
- **MPU6050 Sensor**: Provides tilt angle and acceleration data for balancing.
- **Stepper Motors**: Drive the robot's wheels with precise control.

### Software
- **Python (Flask)**: Backend server for processing navigation decisions and communicating with the robot.
- **React & Node.js**: Frontend web application for real-time maze visualization.
- **C++**: Embedded code for the ESP32 and Arduino Nano.
- **SystemVerilog**: FPGA image processing for color and lane detection.
- **AWS DynamoDB**: Database for storing maze node data and navigation history.



## How It Works

1. **Maze Navigation**:
   - The robot uses an FPGA camera to detect colored beacons and white LED strips, identifying junctions and walls.
   - A Depth-First Search (DFS) algorithm, implemented on the Python server, determines the robot's path through the maze.
   - The ESP32 microcontroller sends junction data to the server via HTTP POST requests and receives navigation instructions.

2. **Real-Time Mapping**:
   - The Python server processes the robot's data and updates the DynamoDB database with maze node information.
   - The React web app fetches this data and displays the robot's live position and the maze layout.

3. **Self-Balancing**:
   - The Arduino Nano uses a cascaded PID controller to maintain the robot's balance on two wheels.
   - The MPU6050 sensor provides real-time tilt angle and acceleration data for the control system.


## Report

- For a comprehensive overview of the technical details and implementation, please refer to the complete project report, `Y2_Group_Project_MazeMaster.pdf`, uploaded in the repository.
  


## Acknowledgments

- Thanks to Imperial College London, Department of Electrical and Electronic Engineering, for providing the resources and guidance for this project.
- Thanks to all the team members for their individual contributions and joint efforts.
