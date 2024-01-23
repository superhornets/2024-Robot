package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
    private final CANSparkMax m_motor = new CANSparkMax(ClimberConstants.kMotorCanId, MotorType.kBrushless);
    private final RelativeEncoder m_encoder = m_motor.getEncoder();

    public ClimberSubsystem() {
        // Initialize anything else that couldn't be initialized yet

        // Configure anything
        m_motor.setInverted(ClimberConstants.kMotorInverted);

        this.setDefaultCommand(new RunCommand(() -> m_motor.set(0)));
    }

    public void set(double speed) {
        m_motor.set(speed);

    }

    @Override
    public void periodic() {
        // Send sensor values and any other telemetry to the Smart Dashboard

        // If there were actually 2 of the same subsystems, take care to differentiate each instance by name.
        // Otherwise, they will fight over the same Smart Dashboard key/name.

        SmartDashboard.putNumber("climbed height (motor rotations)", m_encoder.getPosition());
    }
}
