package frc.robot.Commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterRunAtSpeedCommand extends Command {
    private final ShooterSubsystem m_ShooterSubsystem;
    private double speed = 0;

    public ShooterRunAtSpeedCommand(ShooterSubsystem shooterSubsystem, double speed) {
        addRequirements(shooterSubsystem);
        m_ShooterSubsystem = shooterSubsystem;
        this.speed = speed;
    }

    @Override
    public void execute() {
        m_ShooterSubsystem.runShooterToInput(speed);
    }

    @Override
    public boolean isFinished() {
        //Check if is finished
        return true;
    }
}
