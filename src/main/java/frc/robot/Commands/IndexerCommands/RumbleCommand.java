package frc.robot.Commands.IndexerCommands;

import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.LegSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Subsystem;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;

public class RumbleCommand extends Command {

    private final IndexerSubsystem m_rumble;

    public RumbleCommand(IndexerSubsystem rumble) {
        addRequirements(rumble);
        m_rumble = rumble;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {

        return false;
    }

}
