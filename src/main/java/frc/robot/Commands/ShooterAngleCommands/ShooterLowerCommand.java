package frc.robot.Commands.ShooterAngleCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterAngleSubsystem;

public class ShooterLowerCommand extends Command {
    // Declare subsystem variables
    private final ShooterAngleSubsystem m_angleSubsystem;

    public ShooterLowerCommand(ShooterAngleSubsystem angleSubsystem) {
        addRequirements(angleSubsystem);
        m_angleSubsystem = angleSubsystem;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_angleSubsystem.moveDown();
    }

    @Override
    public void end(boolean interrupted) {
        //double position = m_angleSubsystem.getPosition();
        //m_angleSubsystem.moveTo(position);
    }

    @Override
    public boolean isFinished() {
        // Have we reached our destination?
        return false;
    }
}
