package frc.robot.Commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterStopCommand extends Command {
    private final ShooterSubsystem m_ShooterSubsystem;

    public ShooterStopCommand(ShooterSubsystem shooterSubsystem) {
        addRequirements(shooterSubsystem);
        m_ShooterSubsystem = shooterSubsystem;
    }

    @Override
    public void execute() {
        m_ShooterSubsystem.stopShooter();
    }
}
