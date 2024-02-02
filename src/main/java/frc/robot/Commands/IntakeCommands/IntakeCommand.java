package frc.robot.Commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {
    // Declare subsystem variables
    private final IntakeSubsystem kIntakeSubsystem;


    public IntakeCommand(IntakeSubsystem intake) {
        addRequirements(intake);
        kIntakeSubsystem = intake;
    }

    @Override
    public void initialize() {
        //Initializes intake
    }

    @Override
    public void execute() {
        //Continue intaking
        kIntakeSubsystem.takeIn();

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
