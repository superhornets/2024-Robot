package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LegConstants;
import frc.robot.Constants.ShooterAngleConstants;

public class ShooterAngleSubsystem extends SubsystemBase {
    // Initialize motors and sensors

    private final CANSparkMax m_motor = new CANSparkMax(ShooterAngleConstants.kMotorCanId, MotorType.kBrushless);
    private final RelativeEncoder m_encoder = m_motor.getEncoder();
    private final SparkPIDController m_pidController = m_motor.getPIDController();

    private String m_mode = "Manual";

    public ShooterAngleSubsystem() {
        // Initialize anything else that couldn't be initialized yet

        // Configure anything
        m_motor.setInverted(ShooterAngleConstants.kMotorInverted);

        // If we're not walking, stop! There are other ways to stop too.
        this.setDefaultCommand(new RunCommand(() -> m_motor.set(0), this));
    }

    public void raise() {
        m_motor.set(0.01);
    }

    public void lower() {
        m_motor.set(0.01);
    }

    public void ampAngle() {
        m_motor.set(0.01);
    }

    public void autoAngle() {
        m_motor.set(0.01);
    }

    public void podiumAngle() {
        m_motor.set(0.01);
    }

    public void subwooferAngle() {
        m_motor.set(0.01);
    }

    public boolean isOverExtended() {
        return m_encoder.getPosition() > ShooterAngleConstants.kMaxExtension;
    }

    @Override
    public void periodic() {
        // Send sensor values and any other telemetry to the Smart Dashboard

        // If there were actually 2 of the same subsystems, take care to differentiate each instance by name.
        // Otherwise, they will fight over the same Smart Dashboard key/name.

    }
}
