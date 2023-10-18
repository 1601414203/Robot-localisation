# Particle Filter Localization
The PFLocaliser class implements a Particle Filter for robot localization, with a focus on handling "kidnapping" scenarios where the robot is suddenly displaced. It maintains a cloud of particles to estimate the robot's pose and can globally resample particles in response to detected kidnappings.
# Features
	
    *Particle Filter Algorithm: A statistical method to estimate the robot's pose by propagating a cloud of particles.
    *Kidnap Detection: Ability to detect abrupt changes in the robot's pose that may indicate a kidnapping scenario.
    *Global Resampling: In case of kidnapping detection, a global resampling of particles is performed to recover the correct pose.
    *Modular Structure: Built with a modular structure making it easy to integrate with other ROS (Robot Operating System) based systems.
# Getting Started
Prerequisites

    ROS (Robot Operating System)
    Python
    NumPy library
    
