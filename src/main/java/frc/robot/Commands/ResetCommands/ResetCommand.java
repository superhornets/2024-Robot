package frc.robot.Commands.ResetCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TestSubsystem;

public class ResetCommand extends Command {
    // Declare subsystem variables
    private final TestSubsystem m_arm;

    public ResetCommand(TestSubsystem arm) {
        addRequirements(arm);
        m_arm = arm;
    }

    @Override
    public void initialize() {
        m_arm.resetEncoder();
    }

    @Override
    public void execute() {
        m_arm.moveTo(100);

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
