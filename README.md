# EWolf-BFMC2026
Official repository for Team EWolf participating in the Bosch Future Mobility Challenge (BFMC) 2026. This project focuses on developing autonomous driving algorithms, including lane detection, traffic sign recognition, and vehicle control logic for a 1/10 scale vehicle. We are a group of Mechatronics Engineering students from the **Benemérita Universidad Autónoma de Puebla (BUAP)** in Puebla, Mexico.

## About Us
* **Origin:** Puebla, Mexico.
* **Institution:** BUAP (Facultad de Ciencias de la Electrónica).
* **Focus:** Developing autonomous driving solutions through hardware integration, computer vision, and control theory.

## Current Project Status
As of February 2026:
* The vehicle can autonomously follow lane boundaries using real-time edge-detection vision algorithms processed on the Raspberry Pi 5. 
* It maintains lateral guidance through a Stanley Controller, which minimizes cross-track and heading errors during straight paths and moderate curves. 
* The development stack is mirrored in a ROS/Gazebo environment, allowing for rapid testing of control laws before physical implementation.
* The system currently encounters instability (oscillations) at higher speeds due to the ongoing syntonisation of controller gains.
