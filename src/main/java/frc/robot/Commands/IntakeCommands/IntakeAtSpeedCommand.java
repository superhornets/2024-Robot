package frc.robot.Commands.IntakeCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeAtSpeedCommand extends Command {
    // Declare subsystem variables
    private final IntakeSubsystem m_intakeSubsystem;
    private DoubleSupplier m_intakeSpeedSupplier;

    public IntakeAtSpeedCommand(IntakeSubsystem intake, DoubleSupplier speedSupplier) {
        addRequirements(intake);
        m_intakeSubsystem = intake;
        m_intakeSpeedSupplier = speedSupplier;
    }

    @Override
    public void initialize() {
        //Initializes intake
    }

    @Override
    public void execute() {
        //Continue intaking
        m_intakeSubsystem.takeInAtSpeed(m_intakeSpeedSupplier.getAsDouble());

    }

    @Override
    public void end(boolean interrupted) {
        //End intake
    }

    @Override
    public boolean isFinished() {
        //Check if is finished
        return false;
    }
}
