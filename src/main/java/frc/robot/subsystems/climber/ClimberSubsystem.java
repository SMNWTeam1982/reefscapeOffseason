package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ClimberConstants.CLIMB_STATE;

public class ClimberSubsystem extends SubsystemBase {
  private SparkMax climbMotor;
  private RelativeEncoder motorEncoder;
  private Rotation2d defaultPosition = CLIMB_STATE.LOWERED.position;
  private CLIMB_STATE currentState = CLIMB_STATE.LOWERED;

  private PIDController climbPIDController;

  /**
   * Configures a Climber Subsystem with one climb motor, this may change if another motor is added
   * when we have shop access
   *
   * @param climbMotorID The CAN ID of the Climb Motor
   */
  public ClimberSubsystem(int climbMotorID) {
    climbMotor = new SparkMax(climbMotorID, MotorType.kBrushless);
    climbMotor.configure(
        ClimberConstants.CLIMB_MOTOR_CONFIG,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
    motorEncoder = climbMotor.getEncoder();
    motorEncoder.setPosition(defaultPosition.getRotations());

    climbPIDController =
        new PIDController(
            ClimberConstants.CLIMB_PROPORTIONAL_GAIN,
            ClimberConstants.CLIMB_INTEGRAL_GAIN,
            ClimberConstants.CLIMB_DERIVATIVE_GAIN);

    climbPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void periodic() {
    logClimberData();
  }

  /**
   * Gets the current climb motor state
   *
   * @return {@link Rotation2d} With the current rotation of the climb motor
   */
  private Rotation2d getClimberRotation() {
    double relativePosition = motorEncoder.getPosition() * ClimberConstants.ROTATIONS_TO_DEGREES;
    double positionRadians = MathUtil.angleModulus(Units.degreesToRadians(relativePosition));
    return new Rotation2d(positionRadians);
  }

  /**
   * Returns the current climber state
   * @return current state as a {@link CLIMB_STATE}
   */
  private CLIMB_STATE getClimberState() {
    return currentState;
  }

  /**
   * Sets the climber to the desired state
   *
   * @param desiredState Desired {@link CLIMB_STATE} to set the climber to.
   */
  public void setClimberState(CLIMB_STATE desiredState) {
    if (getClimberState() == desiredState) {
      return;
    }
    double climberOutput =
        climbPIDController.calculate(getClimberRotation().getRadians(), desiredState.position.getRadians());
    // clamp values
    if (climberOutput >= 12.0) {
      climberOutput = 12.0;
    }
    if (climberOutput <= -12.0) {
      climberOutput = 12.0;
    }

    currentState = desiredState;
    climbMotor.setVoltage(climberOutput);
  }

  private void logClimberData() {
    Logger.recordOutput("Climber/State", currentState);
    Logger.recordOutput("Climber/Angle", getClimberRotation().getDegrees());
    Logger.recordOutput("Climber/Motor Current", climbMotor.getOutputCurrent());
    Logger.recordOutput("Climber/Motor Temperature", climbMotor.getMotorTemperature());
  }
}
