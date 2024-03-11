package frc.robot.Commands.ShooterAngleCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterAngleConstants;
import frc.robot.subsystems.ShooterAngleSubsystem;

public class ShooterSubwooferCommand extends Command {

    private final ShooterAngleSubsystem m_angleSubsystem;

    public ShooterSubwooferCommand(ShooterAngleSubsystem angleSubsystem) {
        addRequirements(angleSubsystem);
        m_angleSubsystem = angleSubsystem;
    }

    @Override
    public void initialize() {
        System.out.println("Start shooter angle subwoofer");

    }

    @Override
    public void execute() {
        m_angleSubsystem.moveTo(ShooterAngleConstants.kSubwooferPosition);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Stop shooter to angle subwoofer");

    }

    @Override
    public boolean isFinished() {
        // Have we reached our destination?
        return m_angleSubsystem.isAtSetpoint();
    }
}
