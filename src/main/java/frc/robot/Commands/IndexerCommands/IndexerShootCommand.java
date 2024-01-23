package frc.robot.Commands.IndexerCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;

public class IndexerShootCommand extends Command {
    // Declare subsystem variables
    private final IndexerSubsystem m_indexer;

    // Declare subsystem state (i.e. status) and initialize
    private boolean goodHealth = true;

    public IndexerShootCommand(IndexerSubsystem indexer) {
        addRequirements(indexer);
        m_indexer = indexer;
    }

    @Override
    public void initialize() {
        // This is the moment we go from standing to walking
    }

    @Override
    public void execute() {
        // Continue walking
        if (goodHealth) {
            m_indexer.shoot();
        }

        if (m_indexer.isTriggered()) {
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
