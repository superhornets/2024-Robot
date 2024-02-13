package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {


    private final CANSparkMax m_motorTop = new CANSparkMax(IntakeConstants.kMotorTopCanId, MotorType.kBrushless);
    private final CANSparkMax m_motorBottom = new CANSparkMax(IntakeConstants.kMotorBottomCanId, MotorType.kBrushless);
    private final RelativeEncoder m_encoder = m_motorTop.getEncoder();
    private final BooleanSupplier m_indexerIsTriggeredSupplier;

    public IntakeSubsystem(BooleanSupplier indexerIsTriggered) {

        m_encoder.setVelocityConversionFactor(IntakeConstants.kGearRatio);
        m_motorTop.setInverted(IntakeConstants.kMotorInverted);
        m_motorBottom.setInverted(IntakeConstants.kMotorInverted);
        m_indexerIsTriggeredSupplier = indexerIsTriggered;

        this.setDefaultCommand(new RunCommand(() -> {
            m_motorTop.set(0);
            m_motorBottom.set(0);
        }, this));

    }

    //Run intake
    public void takeIn() {
        if (m_indexerIsTriggeredSupplier.getAsBoolean()) {
            m_motorTop.set(0);
            m_motorBottom.set(0);
        } else {
        m_motorTop.set(IntakeConstants.kIntakeSpeed);
        m_motorBottom.set(IntakeConstants.kIntakeSpeed);
    }
}

    //Run intake
    public void takeInAtSpeed(double speed) {
        double motorSpeed = speed * IntakeConstants.kIntakeAtSpeed;
        m_motorTop.set(motorSpeed);
        m_motorBottom.set(motorSpeed);
    }

    //Run outtake
    public void takeOut() {
        m_motorBottom.set(IntakeConstants.kOuttakeSpeed);
        m_motorTop.set(IntakeConstants.kOuttakeSpeed);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("IntakeRPM", m_encoder.getVelocity());
    }

}
