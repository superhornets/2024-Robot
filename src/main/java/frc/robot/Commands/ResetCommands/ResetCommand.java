package frc.robot.Commands.ResetCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TestConstants;
import frc.robot.subsystems.TestSubsystem;

public class ResetCommand extends Command {
    // Declare subsystem variables
    private final TestSubsystem m_arm;

    private boolean firstLine = true;

    public ResetCommand(TestSubsystem arm) {
        addRequirements(arm);
        m_arm = arm;
    }

    @Override
    public void initialize() {
        firstLine = true;
        System.out.println("{");
        System.out.println("\t\"" + TestConstants.kMode + "\": [");
        m_arm.resetEncoder();
    }

    @Override
    public void execute() {
        m_arm.moveTo(100);
        if (!firstLine) {
            System.out.println(",");
        }
        firstLine = false;
        System.out.println("\t\t{");
        System.out.println("\t\t\t\"test position\": \"" + m_arm.getPosition() + "\",");
        System.out.println("\t\t\t\"test power\": \"" + m_arm.getAppliedOutput() + "\"");
        System.out.print("\t\t}");

    }

    @Override
    public void end(boolean interrupted) {
        System.out.println();
        System.out.println("\t]");
        System.out.println("}");
    }

    @Override
    public boolean isFinished() {
        // Have we reached our destination?
        return false;
    }
}
