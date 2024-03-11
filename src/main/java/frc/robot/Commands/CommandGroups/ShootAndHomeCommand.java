package frc.robot.Commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.IndexerCommands.IndexerShootCommand;
import frc.robot.Commands.ShooterAngleCommands.ShooterAngleHomeCommand;
import frc.robot.Constants.IndexerConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterAngleSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootAndHomeCommand extends SequentialCommandGroup {

    public ShootAndHomeCommand(IndexerSubsystem indexerSubsystem, ShooterAngleSubsystem shooterAngleSubsystem,
            ShooterSubsystem shooterSubsystem) {
        addCommands(new IndexerShootCommand(indexerSubsystem).withTimeout(IndexerConstants.kTime),
                new ShooterAngleHomeCommand(shooterAngleSubsystem));
    }

}
