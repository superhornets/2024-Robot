package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants.LegConstants;

public class LegSubsystem extends SubsystemBase {
    // Initialize motors and sensors

    private final CANSparkMax m_motor = new CANSparkMax(LegConstants.kMotorCanId, MotorType.kBrushless);
    private final RelativeEncoder m_encoder = m_motor.getEncoder();

    public LegSubsystem() {
        // Initialize anything else that couldn't be initialized yet

        // Configure anything
        m_motor.setInverted(LegConstants.kMotorInverted);

        // If we're not walking, stop! There are other ways to stop too.
        this.setDefaultCommand(new RunCommand(() -> m_motor.set(0), this));
    }

    public void extend() {
        m_motor.set(0.01);
    }

    public boolean isOverExtended() {
        return m_encoder.getPosition() > LegConstants.kMaxExtension;
    }

    @Override
    public void periodic() {
        // Send sensor values and any other telemetry to the Smart Dashboard

        // If there were actually 2 of the same subsystems, take care to differentiate each instance by name.
        // Otherwise, they will fight over the same Smart Dashboard key/name.

        SmartDashboard.putNumber("leg bend", m_encoder.getPosition());
    }
}
