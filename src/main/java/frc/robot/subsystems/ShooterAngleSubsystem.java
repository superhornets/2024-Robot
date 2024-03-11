package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterAngleConstants;

public class ShooterAngleSubsystem extends SubsystemBase {
    // Initialize motors and sensors

    private final CANSparkMax m_motor = new CANSparkMax(ShooterAngleConstants.kMotorCanId, MotorType.kBrushless);
    private final SparkPIDController m_pidController = m_motor.getPIDController();
    private final AbsoluteEncoder m_encoder = m_motor.getAbsoluteEncoder(Type.kDutyCycle);
    private final SparkLimitSwitch m_switch = m_motor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
    private double kSetpoint = 0;

    public ShooterAngleSubsystem() {
        // Initialize anything else that couldn't be initialized yet
        m_switch.enableLimitSwitch(false);
        m_motor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
        m_motor.setInverted(ShooterAngleConstants.kMotorInverted);
        m_encoder.setInverted(ShooterAngleConstants.kEncoderInverted);
        m_encoder.setZeroOffset(120);
        m_motor.setSoftLimit(SoftLimitDirection.kForward, ShooterAngleConstants.kSoftLimit);
        m_motor.enableSoftLimit(SoftLimitDirection.kForward, true);

        m_pidController.setP(ShooterAngleConstants.kP);
        m_pidController.setI(ShooterAngleConstants.kI);
        m_pidController.setD(ShooterAngleConstants.kD);

        m_pidController.setFeedbackDevice(m_encoder);
        m_encoder.setPositionConversionFactor(ShooterAngleConstants.kAbsoluteEncoderConversion);
        m_encoder.setVelocityConversionFactor(ShooterAngleConstants.kAbsoluteEncoderConversion);

        m_pidController.setPositionPIDWrappingEnabled(true);
        m_pidController.setPositionPIDWrappingMinInput(ShooterAngleConstants.kAbsoluteEncoderPositionPIDMinInput);
        m_pidController.setPositionPIDWrappingMaxInput(ShooterAngleConstants.kAbsoluteEncoderPositionPIDMaxInput);

        m_pidController.setSmartMotionMinOutputVelocity(ShooterAngleConstants.kMinVelocity, 0);
        m_pidController.setSmartMotionMaxVelocity(ShooterAngleConstants.kMaxVelocity, 0);
        m_pidController.setSmartMotionMaxAccel(ShooterAngleConstants.kMaxAccel, 0);
        m_pidController.setOutputRange(ShooterAngleConstants.kMinOutput, ShooterAngleConstants.kMaxOutput);

        /*this.setDefaultCommand(new RunCommand(() -> {
            holdPosition();
        }, this));*/
    }

    public double getPosition() {
        return m_encoder.getPosition();
    }

    public double getVelocity() {
        return m_encoder.getVelocity();
    }

    public void moveTo(double angle) {
        m_pidController.setReference(angle, ControlType.kPosition);
        kSetpoint = angle;
    }

    public void home() {
        if (m_encoder.getPosition() > 15) {
            moveTo(ShooterAngleConstants.kHomeAboveTen);
            kSetpoint = ShooterAngleConstants.kHomeAboveTen;
        } else {
            m_motor.set(ShooterAngleConstants.kHomeSetDown);
            kSetpoint = Double.NaN;
        }
    }

    public void stop() {
        m_motor.set(0);
    }

    public void moveUp() {
        m_pidController.setReference((m_encoder.getPosition() + 15), ControlType.kPosition);
        kSetpoint = m_encoder.getPosition() + 15;
    }

    public void moveDown() {
        m_pidController.setReference((m_encoder.getPosition() - 15), ControlType.kPosition);
        kSetpoint = m_encoder.getPosition() - 15;
    }

    public void holdPosition() {
        m_pidController.setReference(kSetpoint, ControlType.kPosition);
    }

    public boolean isDown() {
        return m_switch.isPressed();
    }

    public boolean isAtSetpoint() {
        double upperBound = kSetpoint + 3;
        double lowerBound = kSetpoint - 3;
        return (m_encoder.getPosition() > lowerBound) && (m_encoder.getPosition() < upperBound);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter angle position (deg)", getPosition());
        SmartDashboard.putNumber("Shooter angle velocity (deg/sec)", getVelocity());
        SmartDashboard.putNumber("Shooter angle I accumulated", m_pidController.getIAccum());
        SmartDashboard.putNumber("shooter angle output", m_motor.getAppliedOutput());
    }
}
