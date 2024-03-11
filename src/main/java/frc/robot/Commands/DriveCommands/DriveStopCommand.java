package frc.robot.Commands.DriveCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class DriveStopCommand extends Command {
    private final DriveSubsystem m_drive;

    public DriveStopCommand(DriveSubsystem drive) {
        m_drive = drive;
        addRequirements(m_drive);
    }

    @Override
    public void execute() {
        m_drive.drive(0, 0, 0, true, true);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
