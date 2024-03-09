package frc.robot.Commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterAngleSubsystem;

public class IntakeCommand extends Command {
    // Declare subsystem variables
    private final IntakeSubsystem m_intakeSubsystem;
    private final IndexerSubsystem m_indexerSubsystem;
    private final ShooterAngleSubsystem m_ShooterAngleSubsystem;



    public IntakeCommand(IntakeSubsystem intake, IndexerSubsystem indexer,
            ShooterAngleSubsystem shooterAngleSubsystem) {
        addRequirements(intake);
        m_intakeSubsystem = intake;
        m_indexerSubsystem = indexer;
        m_ShooterAngleSubsystem = shooterAngleSubsystem;
    }

    @Override
    public void initialize() {
        //Initializes intake
    }

    @Override
    public void execute() {
        //Continue intaking
        if (!(m_indexerSubsystem.getNoteAcquired() || !m_ShooterAngleSubsystem.isDown())) {
            m_intakeSubsystem.takeIn();
        } else {
            System.out.println("Not Safe To Intake!");
        }

    }

    @Override
    public void end(boolean interrupted) {
        //End intake
    }

    @Override
    public boolean isFinished() {
        //Check if is finished
        return m_indexerSubsystem.getNoteAcquired();
    }
}
