package frc.robot.Commands.ShooterAngleCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterAngleSubsystem;

public class ShooterRaiseCommand extends Command {
    // Declare subsystem variables
    private final ShooterAngleSubsystem m_angleSubsystem;

    public ShooterRaiseCommand(ShooterAngleSubsystem angleSubsystem) {
        addRequirements(angleSubsystem);
        m_angleSubsystem = angleSubsystem;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_angleSubsystem.moveUp();
    }

    @Override
    public void end(boolean interrupted) {
        //double position = m_angleSubsystem.getPosition();
        // m_angleSubsystem.moveTo(position);
    }

    @Override
    public boolean isFinished() {
        // Have we reached our destination?
        return false;
    }
}
