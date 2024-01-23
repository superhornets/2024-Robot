package frc.robot.Commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberAscendCommand extends Command {
    // Declare subsystem variables
    private final ClimberSubsystem m_Climber;

    // Declare subsystem state (i.e. status) and initialize
    private boolean goodHealth = true;

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
        // Continue walking
        if (goodHealth) {
            m_Climber.extend();
        }

        if (m_Climber.isOverExtended()) {
            goodHealth = false;
        }
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
