package frc.robot.Commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberExtendCommand extends Command {
    // Declare subsystem variables
    private final ClimberSubsystem m_climber;

    // Declare subsystem state (i.e. status) and initialize

    public ClimberExtendCommand(ClimberSubsystem climber) {
        addRequirements(climber);
        m_climber = climber;
    }

    @Override
    public void initialize() {
        // This is the moment we go from standing to walking
    }

    @Override
    public void execute() {
        m_climber.set(ClimberConstants.kExtendPower);
    }

    @Override
    public void end(boolean interrupted) {
        // This is the moment we go from walking to standing
    }

    @Override
    public boolean isFinished() {
        // Have we reached our destination?
        return false;
    }
}
