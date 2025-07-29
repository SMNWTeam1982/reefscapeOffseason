// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandStadiaController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.DriveRobotRelative;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.subsystems.vision.PhotonVisionSubsystem;
import frc.robot.subsystems.vision.QuestNavSubsystem;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private PhotonVisionSubsystem photonVision;
  private QuestNavSubsystem questNav;
  private DriveSubsystem driveTrain;

  private final LoggedDashboardChooser<Command> autoChooser;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandStadiaController driverController =
      new CommandStadiaController(OperatorConstants.DRIVER_CONTROLLER_PORT);
  private final CommandJoystick operatorController =
      new CommandJoystick(OperatorConstants.OPERATOR_CONTROLLER_PORT);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  @SuppressWarnings("unused")
  public RobotContainer() {
    if (OperatorConstants.ENABLE_PHOTONLIB && OperatorConstants.ENABLE_QUESTNAV) {
      photonVision =
          new PhotonVisionSubsystem(
              "Front Limelight",
              VisionConstants.PHOTON_CAM_RELATIVE_TO_ROBOT,
              VisionConstants.PHOTON_CAMERA_NAME);
      questNav = new QuestNavSubsystem("Quest Nav");
      driveTrain = new DriveSubsystem(questNav, photonVision);
    } else if (OperatorConstants.ENABLE_PHOTONLIB) {
      photonVision =
          new PhotonVisionSubsystem(
              "Front Limelight",
              VisionConstants.PHOTON_CAM_RELATIVE_TO_ROBOT,
              VisionConstants.PHOTON_CAMERA_NAME);
      driveTrain = new DriveSubsystem(photonVision);
    } else if (OperatorConstants.ENABLE_QUESTNAV) {
      questNav = new QuestNavSubsystem("Quest Nav");
      driveTrain = new DriveSubsystem(questNav);
    } else {
      driveTrain = new DriveSubsystem();
    }

    autoChooser =
        new LoggedDashboardChooser<>("Selected Auto Routine", AutoBuilder.buildAutoChooser());
    // Configure the trigger bindings
    configureDriverBindings();
    configureOperatorBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureDriverBindings() {
    driveTrain.setDefaultCommand(
        new DriveRobotRelative(
            driveTrain,
            () -> -deadZone(driverController.getLeftY()) * 2,
            () -> -deadZone(driverController.getLeftX()) * 2,
            () -> deadZone(driverController.getRightX()) * 3,
            driverController.a()));
  }

  private void configureOperatorBindings() {}

  private double deadZone(double number) {
    if (Math.abs(number) < 0.05) {
      return 0;
    }
    return number;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
