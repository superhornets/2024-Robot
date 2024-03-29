package frc.robot.Commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterRunPodiumCommand extends Command {
    private final ShooterSubsystem m_ShooterSubsystem;

    public ShooterRunPodiumCommand(ShooterSubsystem shooterSubsystem) {
        addRequirements(shooterSubsystem);
        m_ShooterSubsystem = shooterSubsystem;
    }

    @Override
    public void execute() {
        m_ShooterSubsystem.runShooterPodium();
    }

    @Override
    public boolean isFinished() {
        //Check if is finished
        return m_ShooterSubsystem.isAtSpeed();
    }
}
