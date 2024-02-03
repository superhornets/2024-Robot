package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

public class IndexerSubsystem extends SubsystemBase {
    // Initialize motors and sensors

    private final CANSparkMax m_motorRight = new CANSparkMax(IndexerConstants.kMotorRightCanId, MotorType.kBrushless);
    private final CANSparkMax m_motorLeft = new CANSparkMax(IndexerConstants.kMotorLeftCanId, MotorType.kBrushless);

    private final SparkLimitSwitch m_switch = m_motorRight.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);

    public IndexerSubsystem() {
        // Initialize anything else that couldn't be initialized yet

        // Configure anything
        m_motorRight.setInverted(IndexerConstants.kMotorInverted);
        m_motorLeft.setInverted(IndexerConstants.kMotorInverted);

        this.setDefaultCommand(new RunCommand(() -> {
            m_motorLeft.set(0);
            m_motorRight.set(0);
        }, this));

    }

    public boolean isTriggered() {
        return m_switch.isPressed();
    }

    public void intake() {
        if (isTriggered()) {
            m_motorRight.set(0);
            m_motorLeft.set(0);
        } else {
            m_motorRight.set(IndexerConstants.kIntakeSpeed);
            m_motorLeft.set(IndexerConstants.kIntakeSpeed);
        }
    }

    public void reverse() {
        m_motorRight.set(IndexerConstants.kReverseIntakeSpeed);
        m_motorLeft.set(IndexerConstants.kReverseIntakeSpeed);
    }

    public void shoot() {
        m_motorRight.set(IndexerConstants.kIntakeSpeed);
        m_motorLeft.set(IndexerConstants.kIntakeSpeed);
    }

    public boolean getNoteAcquired() {
        return m_switch.isPressed();
    }

    @Override
    public void periodic() {
        // Send sensor values and any other telemetry to the Smart Dashboard

        // If there were actually 2 of the same subsystems, take care to differentiate each instance by name.
        // Otherwise, they will fight over the same Smart Dashboard key/name.

    }
}
