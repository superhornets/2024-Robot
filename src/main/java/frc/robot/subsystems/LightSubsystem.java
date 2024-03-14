package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LightConstants;

public class LightSubsystem extends SubsystemBase {
    private final AddressableLED m_led = new AddressableLED(LightConstants.kPort);
    private final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(LightConstants.kLength);

    public LightSubsystem() {

        // Reuse buffer
        // Default to a length of 60, start empty output
        // Length is expensive to set, so only set it once, then just update data

        m_led.setLength(m_ledBuffer.getLength());

        // Set the data
        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    public void setSpeedLightsOn() {
        for (var i = 0; i < LightConstants.kSplitLength; i++) {
            m_ledBuffer.setRGB(i, 0, 0, 255);
        }
    }

    public void setSpeedLightsOff() {
        for (var i = 0; i < LightConstants.kSplitLength; i++) {
            m_ledBuffer.setRGB(i, 0, 0, 0);
        }

    }

    public void setAngleLightsOn() {
        for (var i = 0; i < LightConstants.kSplitLength; i++) {
            m_ledBuffer.setRGB(i, 255, 0, 0);
        }
    }

    public void setSpeedAndAngleLightsOn() {
        for (var i = 0; i < LightConstants.kSplitLength; i++) {
            if (i < LightConstants.kSplitLength / 2) {
                m_ledBuffer.setRGB(i, 255, 0, 0);
            } else {
                m_ledBuffer.setRGB(i, 0, 0, 255);
            }

        }
    }

    public void setTargetLightsOn() {
        for (var i = LightConstants.kSplitLength; i < LightConstants.kLength; i++) {
            m_ledBuffer.setRGB(i, 0, 255, 0);
        }

    }

    public void setNoteLightsOn() {
        for (var i = LightConstants.kSplitLength; i < LightConstants.kLength; i++) {
            m_ledBuffer.setRGB(i, 255, 255, 0);
        }

    }

    public void setTargetLightsOff() {
        for (var i = LightConstants.kSplitLength; i < LightConstants.kLength; i++) {
            m_ledBuffer.setRGB(i, 0, 0, 0);
        }

    }

    public void updateLights() {
        m_led.setData(m_ledBuffer);
    }
}
