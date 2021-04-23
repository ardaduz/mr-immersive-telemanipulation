# MR Immersive Telemanipulation
Mixed Reality Lab, ETHZ, Fall 2019, http://www.cvg.ethz.ch/teaching/mrlab/

#### Authors:
* Arda Düzçeker
* Jonas Hein
* Qais El Okaili
* Sandro Boccuzzo

During the last decades robots have become more and more omnipresent not only in research labs but all around us. The ubiquitous presence of robots in our daily life brings the focus towards finding a simple, easily applicable and seamlessly integrated interaction methods with such devices. In this report, we discuss how
we can use the concepts of telemanipulation to support seamless human-machine interactions. We focus on the general purpose of using a Microsoft HoloLens 2 as an
untethered mixed reality device to connect and interact with a state of the art collaborative robot such as YuMi from ABB Robotics.

We implement two main types of interaction methods and submodes in each method:

#### 1. External Holographic Operation
   1. Default External Mode
   
       User manipulates the physical robot by dragging the holographic robot arms.
       
       https://user-images.githubusercontent.com/46934354/115934229-d7efe080-a490-11eb-8c3d-89665eacfe61.mp4
       
   2. Task Mode
   
      User constructs a task as a hologram, previews the expected movements, and executes it on the physical robot.
      
      https://user-images.githubusercontent.com/46934354/115934241-dfaf8500-a490-11eb-81c7-a5ae9c17f094.mp4


#### 2. Internal Immersive Operation
   1. Default Internal Mode
      
      Robot mimicks the user's movements, i.e. user's hand and head movements are executed simultaneously on the physical robot.
      
      https://user-images.githubusercontent.com/46934354/115934248-e76f2980-a490-11eb-8f18-3b568e2fa51b.mp4

   2. VR Mode

      On top of default internal mode, user experiences the environment of the physical robot through a video stream.


#### A Very Brief Technical Summary

In the Microsoft HoloLens 2 application:
* Unity, C# and MRTKv2 are used.
* Audio-visual-tactile UI is designed.
* Control messages for the YuMi hand and actuators are generated and sent through ROS#.
* Robot state that is received from the YuMi simulation is visualized.

In the robot-side simulation and application:
* C++ middleware is developed. (Backend is developed at [Computational Robotics Lab](http://crl.ethz.ch/), ETH Zurich)
* Control messages are received and interpreted.
* Inverse kinematics is solved for the target YuMi hand positions.
* States for the robot body parts and joints are sent.
* Stereo images are acquired from Stereolabs ZED Mini camera and sent to the HoloLens.

For further details please refer to the final report and/or contact the authors (also for how to run the code).
