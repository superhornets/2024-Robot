package frc.robot.Commands.DriveCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class DriveSetXCommand extends Command {
    // Declare subsystem variables
    private final DriveSubsystem m_robotDrive;

    public DriveSetXCommand(DriveSubsystem robotDrive) {
        addRequirements(robotDrive);
        m_robotDrive = robotDrive;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_robotDrive.setX();
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
