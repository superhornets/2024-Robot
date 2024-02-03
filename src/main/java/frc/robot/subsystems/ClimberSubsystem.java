package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
    private CANSparkMax m_motor;
    private RelativeEncoder m_encoder;

    public ClimberSubsystem(int canId) {
        // Initialize anything else that couldn't be initialized yetzz
        m_motor = new CANSparkMax(canId, MotorType.kBrushless);
        m_encoder = m_motor.getEncoder();

        // Configure anything
        m_motor.setInverted(ClimberConstants.kMotorInverted);

        this.setDefaultCommand(new RunCommand(() -> {
            m_motor.set(0);
        }, this));
    }

    public void set(double speed) {
        m_motor.set(speed);
    }

    @Override
    public void periodic() {
        // Send sensor values and any other telemetry to the Smart Dashboard

        // If there were actually 2 of the same subsystems, take care to differentiate each instance by name.
        // Otherwise, they will fight over the same Smart Dashboard key/name.

        String label = "motor value" + m_motor.getDeviceId();
        if (m_motor.getDeviceId() == ClimberConstants.kMotorLeftCanId) {
            label = "Left climbed height (motor rotations)";
        } else if (m_motor.getDeviceId() == ClimberConstants.kMotorRightCanId) {
            label = "Right climbed height (motor rotations)";
        }
        SmartDashboard.putNumber(label, m_encoder.getPosition());
    }
}
