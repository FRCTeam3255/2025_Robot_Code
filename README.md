# 2025_Robot_Code - FRC Team 3255's 2025 Robot
Jump into the code [`here!`](src/main/java/frc/robot)


This robot will compete at the following competitions: 
- [Port Hueneme](https://www.thebluealliance.com/event/2025caph)
- [San Diego Regional](https://www.thebluealliance.com/event/2025casd)


## View Our Robots in Action!
- [Instagram](https://www.instagram.com/frcteam3255/) 
- [YouTube (Primary)](https://www.youtube.com/@FRC3255)
- [YouTube (Clips)](https://www.youtube.com/@FRC3255Clips)
- [Build Thread (Chief Delphi)](https://www.chiefdelphi.com/t/frc-3255-supernurds-2025-build-thread/477499)
- [Website](https://supernurds.com)

## What is REEFSCAPE?
![Crescendo Banner](assets/fd_frc_reefscape_wallpaper.png)

REEFSCAPE, presented by Haas, is the 2025 Season of the FIRST Robotics Competition. Each season has its unique objectives and limitations. 
Students have 6 weeks to construct a robot to compete in the season. Once the 6 weeks are up, teams compete at regional or district-level 
competitions (depending on region) to qualify for the FIRST Championship in Houston, Texas.

Interested in learning more about REEFSCAPE? Visit [FIRST's website](https://www.firstinspires.org/robotics/frc/game-and-season) for more details!

## Code Details
Coming soon!
  
## Controls
Coming soon!

## Robot Simulation
Coming Soon!


## Features

### Software

#### Automatic Zeroing
Our Algae Intake Pivot and Elevator automatically zero the encoder when we enable, making us not need an absolute encoder. To do this, the motor slowly runs to hit the mechanism to a hard stop, triggering a current spike and stop in velocity. We detect the current spike and velocity in code to know when the mechanism is at its hard stop in order to zero it correctly.

#### Manual Zeroing
We have code to allow us to manually zero the Algae Intake Pivot and Elevator in order to skip automatic zeroing to save match time. In disabled, a person can quickly zero the Algae Intake Pivot and Elevator by raising them, then hitting the hard stop.

#### Vision Aided Alignment
- Multi-stage system, depending on distance
- Smart; decides which to do via closest
- Double limelights

- **How it works:** Limelights get the robot poses by AprilTags, returning the current pose of the robot. The auto alignment sets different desired positions based on what button the driver chooses, then auto drives or turns to the correct angle to the desired position based on the distance from the robot to the desired position.

#### State Machine Control
- State Machine Diagram
![State Machine](assets\2025_statemachine.png)

- Controller Map
![Controller Map](assets\2025_controllermap.png)

- State Machine link: https://www.tldraw.com/ro/lFqVEhO80IajGo7JezZaz

- The state machine prevents us from going to states before the robot is ready to.

- **State Machine subsystem:** Used to manage different states in the robot. It controls which state transitions between different states. We use enums to control what states we could go to from the current state.
- **States:** Individual commands representing different operational modes of the robot, controlling the behavior of the robot. We set requirements of the commands to be `subStateMachine`.
- **tryState method:** This method runs when we try to go from the current state to the desired state. It returns the states based on the current states and executes the state command.
- **Calling states:** We call the `tryState` method in the `RobotContainer`. We turn whatever it returns into a Deferred Proxy, allowing the `tryState` method to be evaluated multiple times.

#### Scoring Elements Indexing
We have different velocities for coral intake. It goes faster when the coral is in the hopper, and when the sensor in the coral placer senses the coral, it slows down. This helps keep the coral intaked to a constant position.
