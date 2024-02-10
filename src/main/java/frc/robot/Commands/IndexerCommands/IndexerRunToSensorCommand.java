package frc.robot.Commands.IndexerCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;

public class IndexerRunToSensorCommand extends Command {
    private final IndexerSubsystem m_indexer;

    public IndexerRunToSensorCommand(IndexerSubsystem indexer) {
        addRequirements(indexer);
        m_indexer = indexer;
    }

    @Override
    public void initialize() {
        m_indexer.setSwitchEnabled();
    }

    @Override
    public void execute() {
        m_indexer.intake();
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        // Have we reached our destination?
        return false;
    }
}
