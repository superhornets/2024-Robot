package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionAprilTagConstants;

public class VisionAprilTagSubsystem extends SubsystemBase {
    PhotonCamera m_AprilTagCamera = new PhotonCamera("Global_Shutter_Camera");
    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    Transform3d robotToCamera = new Transform3d(
            new Translation3d(VisionAprilTagConstants.kXOffset, VisionAprilTagConstants.kYOffset,
                    VisionAprilTagConstants.kZOffset),
            new Rotation3d(VisionAprilTagConstants.kRollOffset, VisionAprilTagConstants.kPitchOffset,
                    VisionAprilTagConstants.kYawOffset));
    PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, m_AprilTagCamera, robotToCamera);

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }

}
