# DIY-DirectDrive
Software for creating a DirectDrive Wheelbase for simracing, using an ODrive Robotics board and an industrial servo motor.

This Software is for creating a Direct Drive ForceFeedback
Steering Wheel for Driving Simulations.

It connects with an ODrive Robotics Board (https://odriverobotics.com/)
and uses it to send torque commands to a Servo-Motor.
By interfacing with different games, live ForceFeedback data is acquired.

Feature Overview:
    - intuitive graphical user interface

    Simulation of physical effects of real Steering Wheel:
        - friction (constant resistance against movement of the wheel)
        - damping (proportional resistance against movement of the wheel)
        - inertia (proportional resistance against acceleration of the wheel)

        - resistance against further wheel movement when reaching bump-to-bump limits


    Profile Management:
        - all settings can be saved in a profile
        - profiles can be renamed, or deleted
        - profiles will be saved in 'profiles.xml'
        - profiles will be loaded when starting the software


    Data Reconstruction (!!!DISABLED IN CURRENT VERSION!!!):
        - dynamic realtime temporal upsampling of incoming force-feedback
          data to target sample rate of motor controller
        - optional temporal smoothing of reconstructed data


This Software was developed and tested in Python 3.5.3

Packages used in this software:
    - NumPy
    - PyQt5
    - PyQtGraph
    - PYXinput
    - ODrive (https://docs.odriverobotics.com/#downloading-and-installing-tools)
