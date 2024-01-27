package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

    private final CANSparkMax m_motor = new CANSparkMax(IntakeConstants.kMotorCanId, MotorType.kBrushless);
    private final RelativeEncoder m_encoder = m_motor.getEncoder();

    public IntakeSubsystem() {

        m_encoder.setVelocityConversionFactor(IntakeConstants.kGearRatio);
        m_motor.setInverted(IntakeConstants.kMotorInverted);

        this.setDefaultCommand(new RunCommand(() -> m_motor.set(0), this));

    }

    //Run intake
    public void takeIn() {
        m_motor.set(IntakeConstants.kIntakeSpeed);
    }

    //Run outtake
    public void takeOut() {
        m_motor.set(IntakeConstants.kOuttakeSpeed);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("IntakeRPM", m_encoder.getVelocity());
    }

}
