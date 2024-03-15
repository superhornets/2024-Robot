package frc.robot.Commands.DriveCommands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionAprilTagSubsystem;

public class DriveAutoTarget extends PIDCommand {

    private final DriveSubsystem m_driveSubsystem;
    private final VisionAprilTagSubsystem m_visionAprilTagSubsystem;
    private final DoubleSupplier leftStickX;
    private final DoubleSupplier leftStickY;
    private final BooleanSupplier isCanceled;
    private final DoubleSupplier rightStickX;
    private static DoubleSupplier turningSpeed;
    private final BooleanSupplier slowMode;
    private final BooleanSupplier fastMode;
    private final BooleanSupplier robotRelative;

    public DriveAutoTarget(DriveSubsystem drive, VisionAprilTagSubsystem visionAprilTagSubsystem,
            DoubleSupplier leftStickX, DoubleSupplier leftStickY, DoubleSupplier rightStickX,
            BooleanSupplier isCanceled, BooleanSupplier slowMode, BooleanSupplier fastMode,
            BooleanSupplier robotRelative) {
        //m_driveSubsystem = driveSubsystem;
        //m_visionNoteSubsystem = visionNoteSubsystem;
        //addRequirements(driveSubsystem);
        super(new PIDController(ModuleConstants.kDrivingP, ModuleConstants.kDrivingI, ModuleConstants.kDrivingD),
                visionAprilTagSubsystem::getSpeakerYaw, 180, output -> turningSpeed = () -> output, drive);
        getController().enableContinuousInput(-180, 180);
        getController().setTolerance(DriveConstants.kTurnControllerTolerance,
                DriveConstants.kTurnControllerToleranceAcc);
        this.isCanceled = isCanceled;
        this.leftStickX = leftStickX;
        this.leftStickY = leftStickY;
        this.rightStickX = rightStickX;
        this.slowMode = slowMode;
        this.fastMode = fastMode;
        this.robotRelative = robotRelative;
        m_driveSubsystem = drive;
        m_visionAprilTagSubsystem = visionAprilTagSubsystem;

    }

    @Override
    public void execute() {

        super.execute();
        if (m_visionAprilTagSubsystem.hasTargetsAprilTag()) {
            if (m_visionAprilTagSubsystem.isTargetingSpeaker()) {
                if (m_visionAprilTagSubsystem.getDistanceToSpeaker() < 4) {
                    m_driveSubsystem.drive(leftStickY.getAsDouble(), leftStickX.getAsDouble(),
                            turningSpeed.getAsDouble(), true, true);
                } else {
                    m_driveSubsystem.teleOpDrive(leftStickY.getAsDouble(), leftStickX.getAsDouble(),
                            rightStickX.getAsDouble(), !robotRelative.getAsBoolean(), true, slowMode.getAsBoolean(),
                            fastMode.getAsBoolean());
                }
            } else {
                m_driveSubsystem.teleOpDrive(leftStickY.getAsDouble(), leftStickX.getAsDouble(),
                        rightStickX.getAsDouble(), !robotRelative.getAsBoolean(), true, slowMode.getAsBoolean(),
                        fastMode.getAsBoolean());
            }
        } else {
            m_driveSubsystem.teleOpDrive(leftStickY.getAsDouble(), leftStickX.getAsDouble(),
                    rightStickX.getAsDouble(), !robotRelative.getAsBoolean(), true, slowMode.getAsBoolean(),
                    fastMode.getAsBoolean());
        }
    }

    @Override
    public boolean isFinished() {
        // End when the controller is at the reference.
        return isCanceled.getAsBoolean();
    }

}
