package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

import frc.robot.Constants.TestConstants;

public class TestSubsystem extends SubsystemBase {
    // Initialize motors and sensors

    private final CANSparkMax m_motor = new CANSparkMax(21, MotorType.kBrushless);
    private final SparkPIDController m_pidController = m_motor.getPIDController();
    private final RelativeEncoder m_encoder = m_motor.getEncoder();
    private final SparkLimitSwitch m_forwardLimitSwitch = m_motor
            .getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
    private final SparkLimitSwitch m_reverseLimitSwitch = m_motor
            .getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);

    public TestSubsystem() {
        m_motor.restoreFactoryDefaults();
        // Initialize anything else that couldn't be initialized yet

        // Configure anything
        m_motor.setInverted(false);
        m_forwardLimitSwitch.enableLimitSwitch(true);
        m_reverseLimitSwitch.enableLimitSwitch(true);
        m_motor.enableSoftLimit(SoftLimitDirection.kForward, false);
        m_motor.enableSoftLimit(SoftLimitDirection.kReverse, false);

        m_pidController.setP(2);

        m_pidController.setSmartMotionMaxAccel(10000000000.0, 0);
        m_pidController.setSmartMotionMaxVelocity(10000000000.0, 0);

        m_motor.burnFlash();

        // If we're not walking, stop! There are other ways to stop too.
        this.setDefaultCommand(new RunCommand(() -> m_motor.set(0), this));
    }

    public void moveTo(double value) {
        // m_motor.set(value);
        m_pidController.setReference(value, TestConstants.kMode);

    }

    public void resetEncoder() {
        m_encoder.setPosition(0);
    }

    public double getPosition() {
        return m_encoder.getPosition();
    }

    public double getAppliedOutput() {
        return m_motor.getAppliedOutput();
    }

    @Override
    public void periodic() {
        // Send sensor values and any other telemetry to the Smart Dashboard

        // If there were actually 2 of the same subsystems, take care to differentiate each instance by name.
        // Otherwise, they will fight over the same Smart Dashboard key/name.

        SmartDashboard.putNumber("test position", m_encoder.getPosition());
        SmartDashboard.putNumber("test power", m_motor.getAppliedOutput());
        SmartDashboard.putNumber("test encoder", m_encoder.getPosition());

    }
}
