package frc.robot.Commands.IndexerCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;

public class IndexerShootCommand extends Command {
    // Declare subsystem variables
    private final IndexerSubsystem m_indexer;



    public IndexerShootCommand(IndexerSubsystem indexer) {
        addRequirements(indexer);
        m_indexer = indexer;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_indexer.shoot();
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        // Have we reached our destination?
        return true;
    }
}
