package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionNoteSubsystem extends SubsystemBase {
    /*PhotonCamera m_Camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
    
    public PhotonPipelineResult getResultsNote() {
        return m_Camera.getLatestResult();
    }
    
    public boolean hasTargetsNote() {
        return getResultsNote().hasTargets();
    }
    
    public double getBestResultYaw() {
        if (hasTargetsNote()) {
            return getResultsNote().getBestTarget().getYaw();
        } else {
            return 0;
        }
    }*/
}
