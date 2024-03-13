package frc.robot.Commands.ShooterAngleCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterAngleConstants;
import frc.robot.subsystems.ShooterAngleSubsystem;

public class ShooterPodiumCommand extends Command {
    // Declare subsystem variables
    private final ShooterAngleSubsystem m_angleSubsystem;

    public ShooterPodiumCommand(ShooterAngleSubsystem angleSubsystem) {
        addRequirements(angleSubsystem);
        m_angleSubsystem = angleSubsystem;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_angleSubsystem.moveTo(ShooterAngleConstants.kPodiumPosition);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        // Have we reached our destination?
        return m_angleSubsystem.isAtSetpoint();
    }
}
