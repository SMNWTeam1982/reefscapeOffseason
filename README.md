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
