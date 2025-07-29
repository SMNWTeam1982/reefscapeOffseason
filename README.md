# Reefscape Off-Season Repository
[![CI](https://github.com/SMNWTeam1982/reefscapeOffseason/actions/workflows/main.yml/badge.svg)](https://github.com/SMNWTeam1982/reefscapeOffseason/actions/workflows/main.yml)
<br>
This is the off-season code rehash for the 2025 FRC Season Reefscape game. During the season, we depended on a buggy (although better than previous) codebase written in Python. This greatly limited the flexibility of our code and the tools we could use during the season. As this new codebase is based off of the [Java Swerve Template](https://github.com/SMNWTeam1982/java-swerve-template), it has a more solid basis and structure than Python could provide. This codebase also makes the migration to the Command-based paradigm which helps to add greater flexibility and enables the use of tools such as Pathplanner.

## Components
 - AdvantageKit - Advanced logging framework
 - Pathplanner - Comprehensive autonomous
 - PhotonLib - AprilTag/general vision system
 - QuestNav - Odometry data from Quest 3s

## Subsystems
 - Swerve Drive - See the [Java Swerve Template](https://github.com/SMNWTeam1982/java-swerve-template) for more information on this subsystem
 - Vision - See the [Java Swerve Template](https://github.com/SMNWTeam1982/java-swerve-template) for more information on this subsystem
 - Climber - A complete rewrite of the climber code using PID control to hold position among other improvements
 - Elevator - A modified re-implementation of the original elevator subsystem
 - Intake - A complete rewrite of the original intake, seperated into it's own subsystem for better structure


## Driver and Operator Documentation
Although the Driver and Operator controls will likely be taught by the programming team in person, it's important to make note of which binds do what for easy accessibility and code maintenance. The current code is dependent on a Google Stadia controller for both Driver and Operator, but this will change when we enter the season and switch to our proper controllers (I only have a Stadia controller on hand for testing). Despite this, the layout and buttons of the Stadia controller are familiar and compare well with the Xbox. No documentation should need to be changed.

### Driver Controller
Currently, the code base is configured for robot relative drive for testing, although the controls will remain primarily the same. To drive the robot Vertically (X-axis/forward and back), the current bind is the left joystick's vertical axis. For horizontal robot drive (Y-axis/left and right), the current bind is the left joystick's horizontal axis. For the robots rotational movement, the right joystick's horizontal axis is the current bind. The A button on the Driver's controller will reset the robot's heading in code.

### Operator Controller
The Operator controller uses the right bumper to raise the climber. The left bumper will lower the climber.