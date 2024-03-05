package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionNoteSubsystem extends SubsystemBase {
    PhotonCamera m_Camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");

    public PhotonPipelineResult getResults() {
        return m_Camera.getLatestResult();
    }

    public boolean hasTargets() {
        return getResults().hasTargets();
    }

    public double getBestResultYaw() {
        if (hasTargets()) {
            return getResults().getBestTarget().getYaw();
        } else {
            return 0;
        }
    }
}
