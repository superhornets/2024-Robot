package frc.robot.Commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Commands.ShooterAngleCommands.ShooterAngleHomeCommand;
import frc.robot.Commands.ShooterCommands.ShooterStopCommand;
import frc.robot.subsystems.ShooterAngleSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterStopAndHomeCommand extends ParallelCommandGroup {
    public ShooterStopAndHomeCommand(ShooterSubsystem shooterSubsystem, ShooterAngleSubsystem shooterAngleSubsystem) {
        addCommands(new ShooterAngleHomeCommand(shooterAngleSubsystem), new ShooterStopCommand(shooterSubsystem));
    }
}
