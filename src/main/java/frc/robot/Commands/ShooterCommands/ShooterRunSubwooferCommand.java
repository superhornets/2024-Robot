package frc.robot.Commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterRunSubwooferCommand extends Command {
    private final ShooterSubsystem m_ShooterSubsystem;

    public ShooterRunSubwooferCommand(ShooterSubsystem shooterSubsystem) {
        addRequirements(shooterSubsystem);
        m_ShooterSubsystem = shooterSubsystem;
    }

    @Override
    public void initialize() {
        System.out.println("Start shooter to speed subwoofer");
    }

    @Override
    public void execute() {
        m_ShooterSubsystem.runShooterSubwoofer();
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("stop shooter to Speed");
    }
    @Override
    public boolean isFinished() {
        //Check if is finished
        return m_ShooterSubsystem.isAtSpeed();
    }
}
