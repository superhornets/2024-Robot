package frc.robot.Commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class OuttakeCommand extends Command {
    // Declare subsystem variables
    private final IntakeSubsystem m_outtakeSubsystem;

    // Declare subsystem state (i.e. status) and initialize

    public OuttakeCommand(IntakeSubsystem outtake) {
        addRequirements(outtake);
        m_outtakeSubsystem = outtake;
    }

    @Override
    public void initialize() {
        System.out.println("Outtake initialize");
        //Initialize outtaking
    }

    @Override
    public void execute() {
        //Continue outtaking
        m_outtakeSubsystem.takeOut();

    }


    @Override
    public void end(boolean interrupted) {
        System.out.println("Outtake end");
        //End outtaking
    }

    @Override
    public boolean isFinished() {
        //Check if is finished

        return false;
    }
}
