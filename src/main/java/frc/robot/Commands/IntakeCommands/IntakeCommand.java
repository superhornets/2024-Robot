package frc.robot.Commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {
    // Declare subsystem variables
    private final IntakeSubsystem kIntakeSubsystem;

    // Declare subsystem state (i.e. status) and initialize
    private boolean goodHealth = true;

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
        if (goodHealth) {
            kIntakeSubsystem.takeIn();
        }
    }

    @Override
    public void end(boolean interrupted) {
        //End intake
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
