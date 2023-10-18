# Particle Filter Localization
The PFLocaliser class implements a Particle Filter for robot localization, with a focus on handling "kidnapping" scenarios where the robot is suddenly displaced. It maintains a cloud of particles to estimate the robot's pose and can globally resample particles in response to detected kidnappings.
