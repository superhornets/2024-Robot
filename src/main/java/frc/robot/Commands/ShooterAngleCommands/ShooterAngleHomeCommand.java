package frc.robot.Commands.ShooterAngleCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterAngleConstants;
import frc.robot.subsystems.ShooterAngleSubsystem;

public class ShooterAngleHomeCommand extends Command {
    // Declare subsystem variables
    private final ShooterAngleSubsystem m_angleSubsystem;

    public ShooterAngleHomeCommand(ShooterAngleSubsystem angleSubsystem) {
        addRequirements(angleSubsystem);
        m_angleSubsystem = angleSubsystem;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_angleSubsystem.moveTo(0);
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
