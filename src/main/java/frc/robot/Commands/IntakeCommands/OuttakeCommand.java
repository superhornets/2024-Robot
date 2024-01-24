package frc.robot.Commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class OuttakeCommand extends Command {
    // Declare subsystem variables
    private final IntakeSubsystem kOuttakeSubsystem;

    // Declare subsystem state (i.e. status) and initialize
    private boolean goodHealth = true;

    public OuttakeCommand(IntakeSubsystem outtake) {
        addRequirements(outtake);
        kOuttakeSubsystem = outtake;
    }

    @Override
    public void initialize() {
        //Initializes outtake
    }

    @Override
    public void execute() {
        //Continue outtaking
        if (goodHealth) {
            kOuttakeSubsystem.takeOut();
        }
    }

    @Override
    public void end(boolean interrupted) {
        //End outtake
    }

    @Override
    public boolean isFinished() {
        //Check if is finished
        return true;
    }
}
