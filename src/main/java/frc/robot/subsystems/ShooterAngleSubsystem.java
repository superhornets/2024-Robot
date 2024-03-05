package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterAngleConstants;

public class ShooterAngleSubsystem extends SubsystemBase {
    // Initialize motors and sensors

    private final CANSparkMax m_motor = new CANSparkMax(ShooterAngleConstants.kMotorCanId, MotorType.kBrushless);
    private final SparkPIDController m_pidController = m_motor.getPIDController();
    private final AbsoluteEncoder m_encoder = m_motor.getAbsoluteEncoder(Type.kDutyCycle);

    private final TrapezoidProfile m_profile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(ShooterAngleConstants.kMaxVelocity, ShooterAngleConstants.kMaxAccel));
    private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

    private static double kDt = 0.02;

    public ShooterAngleSubsystem() {
        // Initialize anything else that couldn't be initialized yet
        m_motor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
        m_motor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
        m_motor.setInverted(ShooterAngleConstants.kMotorInverted);
        m_encoder.setInverted(ShooterAngleConstants.kEncoderInverted);

        m_pidController.setP(ShooterAngleConstants.kP);
        m_pidController.setI(ShooterAngleConstants.kI);
        m_pidController.setD(ShooterAngleConstants.kD);

        m_pidController.setFeedbackDevice(m_encoder);
        m_encoder.setPositionConversionFactor(ShooterAngleConstants.kAbsoluteEncoderConversion);
        m_encoder.setVelocityConversionFactor(ShooterAngleConstants.kAbsoluteEncoderConversion);

        m_pidController.setPositionPIDWrappingEnabled(true);
        m_pidController.setPositionPIDWrappingMinInput(ShooterAngleConstants.kAbsoluteEncoderPositionPIDMinInput);
        m_pidController.setPositionPIDWrappingMaxInput(ShooterAngleConstants.kAbsoluteEncoderPositionPIDMaxInput);

        //m_pidController.setSmartMotionMinOutputVelocity(ShooterAngleConstants.kMinVelocity, 0);
        //m_pidController.setSmartMotionMaxVelocity(ShooterAngleConstants.kMaxVelocity, 0);
        //m_pidController.setSmartMotionMaxAccel(ShooterAngleConstants.kMaxAccel, 0);
        m_pidController.setOutputRange(ShooterAngleConstants.kMinOutput, ShooterAngleConstants.kMaxOutput);

        this.setDefaultCommand(new RunCommand(() -> {
            holdPosition();
        }, this));

    }

    public double getPosition() {
        return m_encoder.getPosition();
    }

    public double getVelocity() {
        return m_encoder.getVelocity();
    }

    public void moveTo(double angle) {
        //m_pidController.setReference(angle, ControlType.kPosition);

        m_goal = new TrapezoidProfile.State(angle, 0);
        m_setpoint = m_profile.calculate(kDt, m_setpoint, m_goal);
        m_pidController.setReference(m_setpoint.position, ControlType.kPosition);
    }

    public void moveUp() {
        m_pidController.setReference((m_encoder.getPosition() + 15), ControlType.kPosition);
    }

    public void moveDown() {
        m_pidController.setReference((m_encoder.getPosition() - 15), ControlType.kPosition);
    }

    public void holdPosition() {
        m_pidController.setReference(m_encoder.getPosition(), ControlType.kPosition);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter angle position (deg)", getPosition());
        SmartDashboard.putNumber("Shooter angle velocity (deg/sec)", getVelocity());
        SmartDashboard.putNumber("Shooter angle I accumulated", m_pidController.getIAccum());
        SmartDashboard.putNumber("shooter angle output", m_motor.getAppliedOutput());
    }
}
