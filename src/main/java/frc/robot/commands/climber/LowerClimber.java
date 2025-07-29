package frc.robot.commands.climber;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants.CLIMB_STATE;
import frc.robot.subsystems.climber.ClimberSubsystem;

public class LowerClimber extends Command {
    private final ClimberSubsystem climber;
    private final BooleanSupplier run;

    public LowerClimber(ClimberSubsystem climberSubsystem, BooleanSupplier runClimber) {
        climber = climberSubsystem;
        run = runClimber;
        addRequirements(climberSubsystem);
    }

    @Override
    public void execute() {
        if (run.getAsBoolean()) {
            climber.setClimberState(CLIMB_STATE.LOWERED);
        }
    }
}
