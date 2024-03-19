package frc.robot.Commands.ShooterAngleCommands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterAngleSubsystem;
import frc.robot.subsystems.VisionAprilTagSubsystem;

public class ShooterVisionAngleCommand extends Command {
    private final ShooterAngleSubsystem m_angleSubsystem;
    private final VisionAprilTagSubsystem m_visionAprilTagSubsystem;

    private double kdistance;
    private boolean hadTarget;

    public ShooterVisionAngleCommand(VisionAprilTagSubsystem visionAprilTagSubsystem,
            ShooterAngleSubsystem angleSubsystem) {
        addRequirements(angleSubsystem);
        m_visionAprilTagSubsystem = visionAprilTagSubsystem;
        m_angleSubsystem = angleSubsystem;
    }

    @Override
    public void initialize() {
        if (m_visionAprilTagSubsystem.hasTargetsAprilTag()) {
            kdistance = m_visionAprilTagSubsystem.getDistanceToSpeaker();
            hadTarget = true;
        } else {
            hadTarget = false;
            System.out.println("Cannot See Speaker Target. Did Not Start Command");
        }

    }

    @Override
    public void execute() {
        if (hadTarget)
            if (m_visionAprilTagSubsystem.hasTargetsAprilTag()) {
                kdistance = m_visionAprilTagSubsystem.getDistanceToSpeaker();
            } else {
                System.out.println("No Target! Not Updating Shooter Angle!");
            }
        m_angleSubsystem.moveTo(m_angleSubsystem.getShooterSetpointFromTable(kdistance));

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return !hadTarget;
    }
}
