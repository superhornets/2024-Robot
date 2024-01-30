package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

        //this.setDefaultCommand(new RunCommand(() -> m_motor.set(0), this));

        m_pidController.setP(ShooterAngleConstants.kP);
        m_pidController.setI(ShooterAngleConstants.kI);
        m_pidController.setD(ShooterAngleConstants.kD);

        m_encoder.setPositionConversionFactor(5 * 360);
    }

    public double getPosition() {
        return m_encoder.getPosition();
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

    }
}
