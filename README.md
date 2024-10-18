# Robot Dog

Welcome to the Robot Dog project! This repository contains the codebase for controlling a custom-built robot dog designed for autonomous indoor navigation. Our primary development focus is under `robot_ws`, which involves creating ROS2 packages for operating the robot dog.

While the physical robot dog exists, we are currently developing a virtual model to test control algorithms and methodologies before deploying them on the real robot. In parallel, we're designing a custom controller board featuring a powerful STM32 MCU, an IMU, and more.

## Table of Contents

- [Running the Robot Simulation Locally](#running-the-robot-simulation-locally)
- [Development Progress](#development-progress)
  - [Simulation](#simulation)
  - [Control](#control)
  - [Mechanical](#mechanical)
- [Contributing](#contributing)
- [Repo Maintainers](#repo-maintainers)

## Running the Robot Simulation Locally

*Detailed instructions coming soon!*

## Development Progress
### Simulation
To faciltate software development without risking damage to the robot dog we decided to attempt to simulate the physical robot to some degree to de-risk development and make it faster + easier.

#### Matplotlib Model
To begin, an effort was made to create a simple stick figure like model of the robot dog to facilitate developing the IK for individual legs before testing on the physical dog. This model was directly based
on measurements from the dog assembly (in Fusion360) and offered a "quick and dirty" way to verify that the IK looks about right. 

As this model was developed further and the lower level firmware for joint control was established, we began to test and verify this stick figure model along with the IK. These tests involved commanding 
certain leg joint commands and comparing the resulting final leg configuration in real life and the Matplotlib model. These tests allowed us to uncover obvious mistakes
in the IK model and iterate on it until it was fixed. 

![image](https://github.com/user-attachments/assets/9e5253ad-311c-4a26-8528-d4dc2fbd1065)

The model was expanded to involve the full robot body and was used to visualize simple gait sequences genearted using time sinusoidal functions mapped to the x and z positions of the ends of each leg. This
led us to getting a video of the dog taking its first steps 🎉! 

![ezgif-7-b41cf989ee](https://github.com/user-attachments/assets/2a597be4-6f1a-4240-a477-af5c312f57ea)
</br>Clumsy... but we all start somewhere 😅.

#### Gazebo + ROS2
To simplify development and ensure modularity between simulation + testing and physical robot we wanted to use the ROS framework for development, specifically ROS2 (Humble). We found that commonly the general
public seems to use gazebo for ROS2 related projects and thus decided to try it out hoping that there would be enough documentation and support out there (foreshadowing 🥲).

The problem began with having to generate URDF model based on our robot dog. Since our robot was fully designed in Fusion360 we were able to use an open source conversion tool (Fusion2URDF). However, we ran
into a major issue. Due to URDF's tree structure (enforcing a single parent per link) and since our robot used a closed chain design for the leg, we had to find a work-around. After much research, we found that
it would be possible to create essentially generate a URDF model with the leg kinematic chain open then close it using custom software. This was the ideal case as we wanted to be able to check for collisions 
across the entire leg assembly as well as maintain the dynamic properties as much as possible. However, this solution would have involved a lot of work to retrieve the correct relative transforms in fusion
(numbers were not very nice), so we decided to proceed with an initial simplified open chain model for the legs and create a mapper in software that uses the FK of the leg servo joint angles to command the
simulated leg joint angles. We still aim to revist using the open chain model and closing it in softwar in the future as that would provide a better representation of the physical system.

![image (1)](https://github.com/user-attachments/assets/81aa5075-6eb8-4f04-8016-077017ab4275)
</br>Our closed chain 3DOF leg assembly.

![image](https://github.com/user-attachments/assets/b28d3f52-7aac-47ef-b686-1f328761c2e6)
</br>Open chain robot dog assembly to be closed in software.

![image](https://github.com/user-attachments/assets/90abb5cf-f6da-44d5-989f-92a68b089e0a)
</br> Simplified robot dog assembly (in use)

With the URDF model we were able to begin using gazebo for a physics based simulation of our robot dog. After hours of debugging, tuning parameters and correcting the inertia tensors we were able to 
get the robot standing in gazebo. The next step was controlling the dog. For this step we opted for ros2-control as we had seen demonstrations of quadrupeds using ros-control (using ROS1), 
but we were very wrong 💀.

After more hours trying to control the robot using different built-in controllers for ros2-control, turns out, ROS2 control doesn't seem to behave nicely for legged robots (skids and builds up acceleration)...
Who would have thought...

Left with either writing our own controller for the dog in gazebo, which probably would have been to bad looking back at this, but we had finals coming up in just a few weeks at this point and we just wanted to get this done (Calc 3 is not it btw...). Soooo, we decided to redo everything from scratch and write our own physics engine from scratch!!!

![This-is-Fine-300x300](https://github.com/user-attachments/assets/330ddcf8-a036-4f90-bf46-c5fb8c103560)

Just kidding. We just decided to use PyBullet instead. It was mainly just much easier to get setup with our URDF model.

#### PyBullet
PyBullet is pretty straightforward and easy to use which made it really easy to just setup a ROS2 node that spins up a PyBullet instance with a simple world and the dog along with subscribers and publishers
for the joint commands and the robot's state.

At this point we were able to setup launch files and visualization in rviz/foxglove (either is possible currently using foxglove... just felt like it lol...).

Then we just setup a quick control node that carries out IK and FK to determine the URDF model's joint angles based on the servo commanded joint angles to mimic the physical hardware interface as much as possible.

Now we can just test control related work atm and working on getting a few virtual sensors in place starting with IMUs to begin working on the localization and navigation stack.

![27d](https://github.com/user-attachments/assets/a28fa524-6821-42e2-90c6-51e77854569a)

### Controls
Really the goals for this project in terms of control is somewhat limitless (not considering that we are eng students and are too broke to use top of the line hardware). We can really go super simple with 
pre-defined gait sequences or even close loop stabilized gait sequences, or really push our limits and try to implement some cool paper proposing some hybrid MPC + RL approach. Overall, the main goal for
controls is to make the software interface modular enough to allow for any sort of control. Which should be ideally fullfilled by our use of ROS2 nodes to modularize different components of the software.

#### Un-stablized gaits
##### Walk
Essentially one leg lifted at a time while all other legs move backwards (while in contact with the ground) moving the robot forward.
![ezgif-4-aa80b01d10](https://github.com/user-attachments/assets/c1ba8b27-beb9-4015-b9ff-c5e5d7f72cbc)


- Completed task example

#### To-Do

- [ ] To-do task example

### Hardware

#### Done

- Completed task example

#### To-Do

- [ ] To-do task example

### Mechanical

#### Done

- Completed task example

#### To-Do

- [ ] To-do task example
## Contributing

We welcome contributions from all team members. To add updates to the software, hardware, or mechanical sections, please follow these steps:

1. **Fork the repository** and create a new branch for your updates.
2. **Update the relevant section** in the README under [Development Progress](#development-progress).
3. **Commit your changes** with a descriptive message.
4. **Push your branch** to your forked repository.
5. **Create a pull request** to merge your updates into the main repository.

Please ensure your updates are clear and concise, and include any relevant details that would be helpful for the team.

## Repo Maintainers

- **Mostafa Hussein**: [LinkedIn](https://www.linkedin.com/in/mostafa-hussein-04/)
- **Vinesh Vivekanand**: [LinkedIn](https://www.linkedin.com/in/vinesh-vivekanand/)
- **Armaan Rasheed**: [LinkedIn](https://www.linkedin.com/in/armaan-rasheed-530229a0/)

