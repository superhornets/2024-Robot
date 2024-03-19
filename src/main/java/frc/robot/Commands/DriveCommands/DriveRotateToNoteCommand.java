package frc.robot.Commands.DriveCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionAprilTagSubsystem;
import frc.robot.subsystems.VisionNoteSubsystem;

public class DriveRotateToNoteCommand extends PIDCommand {

    //private final DriveSubsystem m_driveSubsystem;
    //private final VisionNoteSubsystem m_visionNoteSubsystem;

    public DriveRotateToNoteCommand(DriveSubsystem drive, VisionAprilTagSubsystem visionAprilTagSubsystem) {
        //m_driveSubsystem = driveSubsystem;
        //m_visionNoteSubsystem = visionNoteSubsystem;
        //addRequirements(driveSubsystem);
        super(new PIDController(ModuleConstants.kDrivingP, ModuleConstants.kDrivingI, ModuleConstants.kDrivingD),
                visionAprilTagSubsystem::getBestResultYaw, 0, output -> drive.drive(0, 0, output, false, false), drive);
        getController().enableContinuousInput(-180, 180);
        getController().setTolerance(DriveConstants.kTurnControllerTolerance,
                DriveConstants.kTurnControllerToleranceAcc);
    }

    @Override
    public boolean isFinished() {
        // End when the controller is at the reference.
        return getController().atSetpoint();
    }

}
