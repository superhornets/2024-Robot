package frc.robot.Commands.IndexerCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IndexerConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IndexerShootCommand extends Command {
    // Declare subsystem variables
    private final IndexerSubsystem m_indexer;
    private final ShooterSubsystem m_shooter;

    private double timeStamp;

    public IndexerShootCommand(IndexerSubsystem indexer, ShooterSubsystem shooter) {
        addRequirements(indexer);
        m_indexer = indexer;
        m_shooter = shooter;
    }

    @Override
    public void initialize() {
        m_indexer.setSwitchDisabled();
        timeStamp = Timer.getFPGATimestamp();
        System.out.println("start Shooting");
    }

    @Override
    public void execute() {
        if (m_shooter.isAtSpeed()) {
            m_indexer.shoot();
        } else {
            System.out.println("Not At Speed!");
        }
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("stop shooting");
        m_indexer.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
