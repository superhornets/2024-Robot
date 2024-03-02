package frc.robot.Commands.ShooterAngleCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterAngleConstants;
import frc.robot.subsystems.ShooterAngleSubsystem;

public class ShooterToAngleCommand extends Command {
    // Declare subsystem variables
    private final ShooterAngleSubsystem m_angleSubsystem;
    private final double kAngle;

    public ShooterToAngleCommand(ShooterAngleSubsystem angleSubsystem, double angle) {
        addRequirements(angleSubsystem);
        m_angleSubsystem = angleSubsystem;
        kAngle = angle;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_angleSubsystem.moveTo(kAngle);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        // Have we reached our destination?
        return false;
    }
}
