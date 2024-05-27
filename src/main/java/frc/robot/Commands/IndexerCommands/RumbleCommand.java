package frc.robot.Commands.IndexerCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;


public class RumbleCommand extends Command {

    private final XboxController m_rumble;
    private double m_startTime;

    public RumbleCommand(XboxController rumble) {
        //addRequirements(rumble);
        m_rumble = rumble;
    }

    @Override
    public void initialize() {
        System.out.println("Initialize rumble");
        m_startTime = Timer.getFPGATimestamp();
        m_rumble.setRumble(RumbleType.kBothRumble, 1.0);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Rumble end");
        m_rumble.setRumble(RumbleType.kBothRumble, 0);
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - m_startTime >= OIConstants.kRumbleTime;
    }

}
