package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionAprilTagSubsystem extends SubsystemBase {
    PhotonCamera m_AprilTagCamera = new PhotonCamera("Global_Shutter_Camera");
}
