package frc.robot.Commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {
    // Declare subsystem variables
    private final IntakeSubsystem m_intakeSubsystem;



    public IntakeCommand(IntakeSubsystem intake) {
        addRequirements(intake);
        m_intakeSubsystem = intake;
    }

    @Override
    public void initialize() {
        System.out.println("Intake initialize");
        //Initializes intake
    }

    @Override
    public void execute() {
        //Continue intaking
        m_intakeSubsystem.takeIn();

    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Intake end");
        //End intake
    }

    @Override
    public boolean isFinished() {
        //Check if is finished
        return false;
    }
}
