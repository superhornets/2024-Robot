package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LegSubsystem;

public class WalkCommand extends Command {
    // Declare subsystem variables
    private final LegSubsystem m_leg;

    // Declare subsystem state (i.e. status) and initialize
    private boolean goodHealth = true;

    public WalkCommand(LegSubsystem leg) {
        addRequirements(leg);
        m_leg = leg;
    }

    @Override
    public void initialize() {
        // This is the moment we go from standing to walking
    }

    @Override
    public void execute() {
        // Continue walking
        if (goodHealth) {
            m_leg.extend();
        }

        if (m_leg.isOverExtended()) {
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
