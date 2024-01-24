package frc.robot.Commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberAscendCommand extends Command {
    // Declare subsystem variables
    private final ClimberSubsystem m_Climber;

    // Declare subsystem state (i.e. status) and initialize

    public ClimberAscendCommand(ClimberSubsystem Climber) {
        addRequirements(Climber);
        m_Climber = Climber;
    }

    @Override
    public void initialize() {
        // This is the moment we go from standing to walking
    }

    @Override
    public void execute() {
        m_Climber.set(ClimberConstants.kAscendPower);
    }

    @Override
    public void end(boolean interrupted) {
        // This is the moment we go from walking to standing
    }

    @Override
    public boolean isFinished() {
        // Have we reached our destination?
        return true;
    }
}
