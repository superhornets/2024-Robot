package frc.robot.Commands.IndexerCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IndexerConstants;
import frc.robot.subsystems.IndexerSubsystem;

public class IndexerShootCommand extends Command {
    // Declare subsystem variables
    private final IndexerSubsystem m_indexer;

    private double timeStamp;

    public IndexerShootCommand(IndexerSubsystem indexer) {
        addRequirements(indexer);
        m_indexer = indexer;
    }

    @Override
    public void initialize() {
        m_indexer.setSwitchDisabled();
        timeStamp = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        m_indexer.shoot();
    }

    @Override
    public void end(boolean interrupted) {
        m_indexer.stop();
    }

    @Override
    public boolean isFinished() {
        if ((Timer.getFPGATimestamp() - timeStamp) >= IndexerConstants.kTime) {
            return true;
        }

        return false;
    }
}
