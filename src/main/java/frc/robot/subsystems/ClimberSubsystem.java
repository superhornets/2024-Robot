package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
    private final CANSparkMax m_motorRight = new CANSparkMax(ClimberConstants.kMotorRightCanId, MotorType.kBrushless);
    private final CANSparkMax m_motorLeft = new CANSparkMax(ClimberConstants.kMotorLeftCanId, MotorType.kBrushless);
    private final RelativeEncoder m_encoder = m_motorRight.getEncoder();

    public ClimberSubsystem() {
        // Initialize anything else that couldn't be initialized yet

        // Configure anything
        m_motorRight.setInverted(ClimberConstants.kMotorInverted);

        this.setDefaultCommand(new RunCommand(() -> {
            m_motorRight.set(0);
            m_motorLeft.set(0);
        }, this));
    }

    public void set(double speed) {
        m_motorRight.set(speed);

    }

    @Override
    public void periodic() {
        // Send sensor values and any other telemetry to the Smart Dashboard

        // If there were actually 2 of the same subsystems, take care to differentiate each instance by name.
        // Otherwise, they will fight over the same Smart Dashboard key/name.

        SmartDashboard.putNumber("climbed height (motor rotations)", m_encoder.getPosition());
    }
}
