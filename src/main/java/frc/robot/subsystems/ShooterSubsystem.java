package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    private final CANSparkMax m_leftMotor = new CANSparkMax(ShooterConstants.kMotorLeftCanId, MotorType.kBrushless);
    private final CANSparkMax m_rightMotor = new CANSparkMax(ShooterConstants.kMotorRightCanId, MotorType.kBrushless);
    private final SparkPIDController m_leftPIDController = m_leftMotor.getPIDController();
    private final SparkPIDController m_rightPIDController = m_rightMotor.getPIDController();
    private final RelativeEncoder m_leftEncoder = m_leftMotor.getEncoder();
    private final RelativeEncoder m_rightEncoder = m_rightMotor.getEncoder();

    public ShooterSubsystem() {
        m_leftMotor.setInverted(ShooterConstants.isLeftMotorInverted);
        m_rightMotor.setInverted(ShooterConstants.isRightMotorInverted);

        m_leftPIDController.setP(ShooterConstants.shooterP);
        m_leftPIDController.setI(ShooterConstants.shooterI);
        m_leftPIDController.setD(ShooterConstants.shooterD);
        m_leftPIDController.setFF(ShooterConstants.shooterFF);
        m_leftPIDController.setOutputRange(ShooterConstants.shooterMin, ShooterConstants.shooterMax);
        m_rightPIDController.setP(ShooterConstants.shooterP);
        m_rightPIDController.setI(ShooterConstants.shooterI);
        m_rightPIDController.setD(ShooterConstants.shooterD);
        m_rightPIDController.setFF(ShooterConstants.shooterFF);
        m_rightPIDController.setOutputRange(ShooterConstants.shooterMin, ShooterConstants.shooterMax);
    }

    public void runShooterSubwoofer() {
        m_leftPIDController.setReference(ShooterConstants.shooterSpeedSubwoofer, ControlType.kVelocity);
        m_rightPIDController.setReference(ShooterConstants.shooterSpeedSubwoofer, ControlType.kVelocity);
    }

    public void runShooterPodium() {
        m_leftPIDController.setReference(ShooterConstants.shooterSpeedPodium, ControlType.kVelocity);
        m_rightPIDController.setReference(ShooterConstants.shooterSpeedPodium, ControlType.kVelocity);
    }

    public void runShooterAmp() {
        m_leftPIDController.setReference(ShooterConstants.shooterSpeedAmp, ControlType.kVelocity);
        m_rightPIDController.setReference(ShooterConstants.shooterSpeedAmp, ControlType.kVelocity);
    }

    public void stopShooter() {
        m_leftPIDController.setReference(0, ControlType.kVelocity);
        m_rightPIDController.setReference(0, ControlType.kVelocity);
    }

    public void runShooterToInput(double speed) {
        m_leftPIDController.setReference(speed, ControlType.kVelocity);
        m_rightPIDController.setReference(speed, ControlType.kVelocity);
    }
   

    
}
