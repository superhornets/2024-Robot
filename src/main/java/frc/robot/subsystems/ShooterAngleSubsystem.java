package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
//import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterAngleConstants;

public class ShooterAngleSubsystem extends SubsystemBase {
    // Initialize motors and sensors

    private final CANSparkMax m_motor = new CANSparkMax(ShooterAngleConstants.kMotorCanId, MotorType.kBrushless);
    // private final RelativeEncoder m_encoder = m_motor.getEncoder();
    private final SparkPIDController m_pidController = m_motor.getPIDController();
    private final AbsoluteEncoder m_encoder = m_motor.getAbsoluteEncoder(Type.kDutyCycle);

    public ShooterAngleSubsystem() {
        // Initialize anything else that couldn't be initialized yet

        // Configure anything
        m_motor.setInverted(ShooterAngleConstants.kMotorInverted);

        m_pidController.setP(ShooterAngleConstants.kP);
        m_pidController.setI(ShooterAngleConstants.kI);
        m_pidController.setD(ShooterAngleConstants.kD);

        // m_encoder.setPositionConversionFactor(5 * 360);

        m_pidController.setFeedbackDevice(m_encoder);
        m_encoder.setPositionConversionFactor(ShooterAngleConstants.kAbsoluteEncoderConversion);
        m_encoder.setVelocityConversionFactor(ShooterAngleConstants.kAbsoluteEncoderConversion);

        m_pidController.setPositionPIDWrappingEnabled(true);
        m_pidController.setPositionPIDWrappingMinInput(ShooterAngleConstants.kAbsoluteEncoderPositionPIDMinInput);
        m_pidController.setPositionPIDWrappingMaxInput(ShooterAngleConstants.kAbsoluteEncoderPositionPIDMaxInput);
    }

    public double getPosition() {
        return m_encoder.getPosition();
    }

    public double getVelocity() {
        return m_encoder.getVelocity();
    }

    public void moveTo(double angle) {

        m_pidController.setReference(angle, ControlType.kPosition);
    }

    public void moveUp() {
        m_motor.set(ShooterAngleConstants.kRaiseSpeed);
    }

    public void moveDown() {
        m_motor.set(ShooterAngleConstants.kLowerSpeed);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter angle position (deg)", getPosition());
        SmartDashboard.putNumber("Shooter angle velocity (deg/sec)", getVelocity());

    }
}
