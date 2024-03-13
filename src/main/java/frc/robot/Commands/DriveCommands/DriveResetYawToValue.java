package frc.robot.Commands.DriveCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class DriveResetYawToValue extends Command {
    // Declare subsystem variables
    private final DriveSubsystem m_robotDrive;
    private final double kPos;

    public DriveResetYawToValue(DriveSubsystem robotDrive, double pos) {
        addRequirements(robotDrive);
        m_robotDrive = robotDrive;
        kPos = pos;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_robotDrive.resetNavXToPos(kPos);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
