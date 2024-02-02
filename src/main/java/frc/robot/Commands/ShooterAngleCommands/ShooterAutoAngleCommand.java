package frc.robot.Commands.ShooterAngleCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterAngleConstants;
import frc.robot.subsystems.ShooterAngleSubsystem;

public class ShooterAutoAngleCommand extends Command {
    private final ShooterAngleSubsystem m_angleSubsystem;

    private double m_angle;

    public ShooterAutoAngleCommand(double angle, ShooterAngleSubsystem angleSubsystem) {
        addRequirements(angleSubsystem);
        m_angle = angle;
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
        return (m_angleSubsystem.getPosition() - m_angle) < ShooterAngleConstants.kAngle;
    }
}
