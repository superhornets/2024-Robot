package frc.robot.Commands.ShooterAngleCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterAngleSubsystem;
import frc.robot.subsystems.VisionAprilTagSubsystem;

public class ShooterVisionAngleCommand extends Command {
    private final ShooterAngleSubsystem m_angleSubsystem;
    private final VisionAprilTagSubsystem m_visionAprilTagSubsystem;

    private double m_angle;

    public ShooterVisionAngleCommand(VisionAprilTagSubsystem visionAprilTagSubsystem,
            ShooterAngleSubsystem angleSubsystem) {
        addRequirements(angleSubsystem);
        m_visionAprilTagSubsystem = visionAprilTagSubsystem;
        m_angleSubsystem = angleSubsystem;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_angleSubsystem.moveTo(m_angle);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
