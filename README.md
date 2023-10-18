# Particle Filter Localization
The PFLocaliser class implements a Particle Filter for robot localization, with a focus on handling "kidnapping" scenarios where the robot is suddenly displaced. It maintains a cloud of particles to estimate the robot's pose and can globally resample particles in response to detected kidnappings.
# Features

#### Particle Filter Algorithm
A statistical method to estimate the robot's pose by propagating a cloud of particles.

#### Kidnap Detection
Ability to detect abrupt changes in the robot's pose that may indicate a kidnapping scenario.

#### Global Resampling
In case of kidnapping detection, a global resampling of particles is performed to recover the correct pose.

#### Modular Structure
Built with a modular structure making it easy to integrate with other ROS (Robot Operating System) based systems.
# Getting Started
#### Prerequisites

    ROS (Robot Operating System)
    Python
    NumPy library
    
# Installation

#### Clone the repository to your local machine, navigate to the project directory, and execute the following commands to get started:
    $ cd path/to/project
    $ pip install -r requirements.txt
    $ rosrun your_package pf_localiser.py
# Usage

#### Initialise Particle Cloud:
        Initialize the particle cloud based on the initial pose estimate.
        Update Particle Cloud:
        Update the particle cloud based on the latest sensor readings.
        Estimate Pose:
        Estimate the robot's pose by computing the weighted mean of the particle cloud.
    Detect Kidnap:
        Check for abrupt changes in
# Contributing

Feel free to fork the project, create a new branch, and submit your pull requests.
License

This project is licensed under the MIT License. pose and trigger global resampling if kidnapping is detected.
        
