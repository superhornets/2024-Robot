package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    private final CANSparkMax m_leftMotor = new CANSparkMax(ShooterConstants.kMotorLeftCanId, MotorType.kBrushless);
    private final CANSparkMax m_rightMotor = new CANSparkMax(ShooterConstants.kMotorRightCanId, MotorType.kBrushless);
    private final SparkPIDController m_leftPIDController = m_leftMotor.getPIDController();
    private final SparkPIDController m_rightPIDController = m_rightMotor.getPIDController();
    private final RelativeEncoder m_leftEncoder = m_leftMotor.getEncoder();
    private final RelativeEncoder m_rightEncoder = m_rightMotor.getEncoder();
    private double setpoint;

    public ShooterSubsystem() {
        m_leftMotor.setInverted(ShooterConstants.kIsLeftMotorInverted);
        m_rightMotor.setInverted(ShooterConstants.kIsRightMotorInverted);

        m_leftPIDController.setP(ShooterConstants.kShooterP);
        m_leftPIDController.setI(ShooterConstants.kShooterI);
        m_leftPIDController.setD(ShooterConstants.kShooterD);
        m_leftPIDController.setFF(ShooterConstants.kShooterFF);
        m_leftPIDController.setOutputRange(ShooterConstants.kShooterMin, ShooterConstants.kShooterMax);
        m_rightPIDController.setP(ShooterConstants.kShooterP);
        m_rightPIDController.setI(ShooterConstants.kShooterI);
        m_rightPIDController.setD(ShooterConstants.kShooterD);
        m_rightPIDController.setFF(ShooterConstants.kShooterFF);
        m_rightPIDController.setOutputRange(ShooterConstants.kShooterMin, ShooterConstants.kShooterMax);
    }

    public void runShooterSubwoofer() {
        m_leftPIDController.setReference(ShooterConstants.kShooterSpeedSubwoofer, ControlType.kVelocity);
        m_rightPIDController.setReference(ShooterConstants.kShooterSpeedSubwoofer, ControlType.kVelocity);
        setpoint = ShooterConstants.kShooterSpeedSubwoofer;
    }

    public void runShooterPodium() {
        m_leftPIDController.setReference(ShooterConstants.kShooterSpeedPodium, ControlType.kVelocity);
        m_rightPIDController.setReference(ShooterConstants.kShooterSpeedPodium, ControlType.kVelocity);
        setpoint = ShooterConstants.kShooterSpeedPodium;
    }

    public void runShooterAmp() {
        m_leftPIDController.setReference(ShooterConstants.kShooterSpeedAmp, ControlType.kVelocity);
        m_rightPIDController.setReference(ShooterConstants.kShooterSpeedAmp, ControlType.kVelocity);
        setpoint = ShooterConstants.kShooterSpeedAmp;
    }

    public void stopShooter() {
        m_leftPIDController.setReference(0, ControlType.kVelocity);
        m_rightPIDController.setReference(0, ControlType.kVelocity);
        setpoint = -99;
    }

    public void runShooterToInput(double speed) {
        m_leftPIDController.setReference(speed, ControlType.kVelocity);
        m_rightPIDController.setReference(speed, ControlType.kVelocity);
    }

    public boolean isAtSpeed() {
        double lowerBound = setpoint - 100;
        double upperBound = setpoint + 100;

        return ((m_leftEncoder.getVelocity() > lowerBound) && (m_leftEncoder.getVelocity() < upperBound))
                && ((m_rightEncoder.getVelocity() > lowerBound) && (m_rightEncoder.getVelocity() < upperBound));
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("velocity", m_leftEncoder.getVelocity());
        SmartDashboard.putBoolean("is at speed", isAtSpeed());

    }

}
