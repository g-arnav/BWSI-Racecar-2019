# BWSI-Racecar-2019
Solutions for the various challenges of Beaverworks Summer Institute 2019 Autonomous Racecar Course

## Week 1 - Intro Week
- Became familiar with ROS
- Learned various control theory concpets such as potential fields, PID, etc
- Implemented a random sample consensus (RANSAC) algorithm based on https://en.wikipedia.org/wiki/Random_sample_consensus

## Week 2 - Using CV for navigation
- Tasked with deciphering the direction of a one way sign and navigating toward an orange cone using the camera
- Devised a system to collect data to train a neural network that finds the angle and distance of the cone from the car based on a picture of the cone
- Since we could not succesfully install tensorflow on the car's arm-based OS, I wrote a python script that interpreted tensoflow's protobuf (.pb) format to build a model that ran natively in python without tensorflow

## Week 3 - More CV and Mapping
- Learned about Localization, Mapping and SLAM algorithms
- Utilized these to map and navigate through a static environment
- Created a line follower algorithm to track and optimize a path with multi colored lines

## Week 4 - Final Challenge
- Lead a team of 5 of my peers in the course's final race
- Implemented all of the concepts listed above to navigate through a stadium sized course filled with various obstacles
- Used AR tags and localization to switch between algorithms
